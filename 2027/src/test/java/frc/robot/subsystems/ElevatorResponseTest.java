package frc.robot.subsystems;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIfEnvironmentVariable;
import org.wpilib.backend.NetworkTablesTelemetryBackend;
import org.wpilib.backend.NetworkTablesTunableBackend;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.networktables.DoubleSubscriber;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.telemetry.TelemetryRegistry;
import org.wpilib.tunable.TunableRegistry;
import frc.robot.Robot;

/**
 * Headless check of the elevator's position controller. Drives the Elevator subsystem through a
 * fixed sequence of targets (the C/V buttons), records the response, prints a summary, and writes
 * a CSV to build/response/. This doesn't pass or fail on controller quality; it measures it.
 *
 * <p>Phoenix 6 simulates the motor on wall-clock time, so this runs in real time: the carriage
 * first settles to the floor while disabled, then each target runs for 5 s.
 *
 * <p>It's slow, so it's skipped by normal builds and deploys. Run it with:
 *
 * <pre>RESPONSE_TEST=1 ./gradlew test --tests '*ElevatorResponseTest' --rerun</pre>
 *
 * <p>To try other gains, set tunables the way the Sim GUI would (comma-separated):
 *
 * <pre>RESPONSE_TUNABLES="kP=100" RESPONSE_TEST=1 ./gradlew test ...</pre>
 */
@EnabledIfEnvironmentVariable(named = "RESPONSE_TEST", matches = "1")
public class ElevatorResponseTest {
  private static final double kMaxSettleSeconds = 15.0;
  private static final double kSecondsPerTarget = 5.0;
  private static final double[] kTargets = {0.5, 1.5, 0.5};

  /** Profile position is NaN at checkpoints before the elevator follows a motion profile. */
  private record Sample(double time, double target, double position, double profile) {}

  private Elevator m_elevator;
  private final DoubleSubscriber m_profilePosition =
      NetworkTableInstance.getDefault()
          .getDoubleTopic("/Telemetry/Elevator/Profile Position (m)")
          .subscribe(Double.NaN);
  private final long m_startNanos = System.nanoTime();
  private long m_nextStepNanos = 0;

  @BeforeEach
  void setUp() {
    HAL.initialize();
    DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
    DriverStationSim.setEnabled(false);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.notifyNewData();
    // RobotBase normally connects these to NetworkTables at startup: tunables (so the test can
    // change them there) and telemetry (so the test can read logged values like the profile)
    TunableRegistry.registerBackend(
        "", new NetworkTablesTunableBackend(NetworkTableInstance.getDefault(), "/Tunables"));
    TelemetryRegistry.registerBackend(
        "", new NetworkTablesTelemetryBackend(NetworkTableInstance.getDefault(), "/Telemetry"));
    m_elevator = new Elevator();
    setTunables();
  }

  /** Publishes RESPONSE_TUNABLES (e.g. "kP=100,kD=3") to /Tunables/, like editing them in NT. */
  private static void setTunables() {
    for (String tunable : tunablesSetting().split(",")) {
      if (!tunable.isBlank()) {
        final String[] nameValue = tunable.split("=");
        NetworkTableInstance.getDefault()
            .getDoubleTopic("/Tunables/" + nameValue[0].trim())
            .publish()
            .set(Double.parseDouble(nameValue[1].trim()));
      }
    }
  }

  private static String tunablesSetting() {
    final String setting = System.getenv("RESPONSE_TUNABLES");
    return setting == null ? "" : setting.replace(" ", "");
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    TunableRegistry.reset();
  }

  @Test
  void positionResponse() throws Exception {
    final List<Sample> samples = new ArrayList<>();
    final int stepsPerTarget = (int) Math.round(kSecondsPerTarget / Robot.periodSecs);

    // Like the Sim GUI, start disabled so gravity settles the carriage onto the floor
    final int maxSettleSteps = (int) Math.round(kMaxSettleSeconds / Robot.periodSecs);
    // Phoenix reports 0 m until it has seen the sim state (the sim starts the carriage up
    // high), so wait to see it above the floor before waiting for it to come down
    boolean seenAboveFloor = false;
    for (int i = 0; i < maxSettleSteps; i++) {
      step();
      final double position = m_elevator.getCurrentPosition();
      seenAboveFloor |= position > 0.005;
      if (seenAboveFloor && position <= 0.005) {
        break;
      }
    }
    final double settleSeconds = m_nextStepNanos / 1e9;
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    final double start = settleSeconds;
    if (!tunablesSetting().isEmpty()) {
      System.out.println("Tunables: " + tunablesSetting());
    }
    System.out.printf("Settled at %.3f m after %.1f s disabled%n",
        m_elevator.getCurrentPosition(), settleSeconds);
    for (double target : kTargets) {
      CommandScheduler.getInstance().schedule(m_elevator.setPositionCommand(target));
      for (int i = 0; i < stepsPerTarget; i++) {
        Unmanaged.feedEnable(100);
        step();
        samples.add(
            new Sample(
                m_nextStepNanos / 1e9 - start,
                target,
                m_elevator.getCurrentPosition(),
                m_profilePosition.get()));
      }
    }

    writeCsv(samples);
    printSummary(samples, stepsPerTarget);
  }

  /** Runs one robot loop, then waits until the next period starts (Phoenix sims in real time). */
  private void step() throws InterruptedException {
    DriverStationBackend.refreshData();
    CommandScheduler.getInstance().run();
    TunableRegistry.update(); // the robot loop does this after robotPeriodic()

    m_nextStepNanos += (long) (Robot.periodSecs * 1e9);
    final long sleepNanos = m_startNanos + m_nextStepNanos - System.nanoTime();
    if (sleepNanos > 0) {
      Thread.sleep(sleepNanos / 1_000_000, (int) (sleepNanos % 1_000_000));
    }
  }

  private static void printSummary(List<Sample> samples, int stepsPerTarget) {
    final int settleWindow = (int) Math.round(2.0 / Robot.periodSecs);
    double previousTarget = samples.get(0).position();

    System.out.printf("%n%-14s %10s %12s %16s %14s%n",
        "Move", "Rise (s)", "Overshoot", "Last 2s p-p", "Last 2s err");
    for (int seg = 0; seg < samples.size() / stepsPerTarget; seg++) {
      final List<Sample> segment =
          samples.subList(seg * stepsPerTarget, (seg + 1) * stepsPerTarget);
      final double target = segment.get(0).target();
      final double startTime = segment.get(0).time();
      final double direction = Math.signum(target - segment.get(0).position());

      // Rise: first time within 2 cm of the target
      double rise = Double.NaN;
      double overshoot = 0.0;
      for (Sample s : segment) {
        if (Double.isNaN(rise) && Math.abs(s.position() - target) <= 0.02) {
          rise = s.time() - startTime;
        }
        overshoot = Math.max(overshoot, direction * (s.position() - target));
      }

      // Steady state: peak-to-peak swing and mean error over the last 2 seconds
      double min = Double.MAX_VALUE;
      double max = -Double.MAX_VALUE;
      double errorSum = 0.0;
      for (Sample s : segment.subList(segment.size() - settleWindow, segment.size())) {
        min = Math.min(min, s.position());
        max = Math.max(max, s.position());
        errorSum += s.position() - target;
      }

      System.out.printf("%-14s %10s %10.1f cm %13.1f cm %+11.1f cm%n",
          String.format("%.1f -> %.1f m", previousTarget, target),
          Double.isNaN(rise) ? "never" : String.format("%.2f", rise),
          overshoot * 100, (max - min) * 100, errorSum / settleWindow * 100);
      previousTarget = target;
    }
  }

  private static void writeCsv(List<Sample> samples) throws IOException {
    final Path dir = Path.of("build", "response");
    Files.createDirectories(dir);
    final String tunables = tunablesSetting().replace(',', '_');
    final Path file =
        dir.resolve(checkpointName() + (tunables.isEmpty() ? "" : "_" + tunables) + ".csv");
    try (PrintWriter out = new PrintWriter(Files.newBufferedWriter(file))) {
      out.println("time_s,target_m,position_m,profile_m");
      for (Sample s : samples) {
        out.printf("%.3f,%.3f,%.4f,%.4f%n", s.time(), s.target(), s.position(), s.profile());
      }
    }
    System.out.println("Wrote " + file.toAbsolutePath());
  }

  /** Names the CSV after the checked-out checkpoint tag, e.g. "2027-bang-bang" or "...-dirty". */
  private static String checkpointName() {
    try {
      final Process git =
          new ProcessBuilder("git", "describe", "--tags", "--always", "--dirty").start();
      final String name = new String(git.getInputStream().readAllBytes()).trim();
      return git.waitFor() == 0 && !name.isEmpty() ? name : "current";
    } catch (IOException | InterruptedException e) {
      return "current";
    }
  }
}
