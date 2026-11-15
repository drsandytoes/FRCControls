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
 * Headless check of the arm's position control, like ElevatorResponseTest: drives the Arm through
 * horizontal, +45°, straight up, -45°, and back (the C/W/V/S keys), prints where each move
 * settled, and writes a CSV (in degrees) to build/response/.
 *
 * <pre>RESPONSE_TEST=1 ./gradlew test --tests '*ArmResponseTest' --rerun</pre>
 *
 * <p>Set tunables the way the Sim GUI would: RESPONSE_TUNABLES="Arm/kP=20,Arm/kG=0".
 */
@EnabledIfEnvironmentVariable(named = "RESPONSE_TEST", matches = "1")
public class ArmResponseTest {
  private static final double kMaxSettleSeconds = 5.0;
  private static final double kSecondsPerTarget = 5.0;
  private static final double[] kTargets = {0.0, 45.0, 90.0, -45.0, 0.0};

  private record Sample(double time, double target, double angle, double profile) {}

  private Arm m_arm;
  private final DoubleSubscriber m_profileAngle =
      NetworkTableInstance.getDefault()
          .getDoubleTopic("/Telemetry/Arm/Profile Angle (deg)")
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
    // RobotBase normally connects these to NetworkTables at startup
    TunableRegistry.registerBackend(
        "", new NetworkTablesTunableBackend(NetworkTableInstance.getDefault(), "/Tunables"));
    TelemetryRegistry.registerBackend(
        "", new NetworkTablesTelemetryBackend(NetworkTableInstance.getDefault(), "/Telemetry"));
    m_arm = new Arm();
    setTunables();
  }

  /** Publishes RESPONSE_TUNABLES (e.g. "Arm/kP=20") to /Tunables/, like editing them in NT. */
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
  void angleResponse() throws Exception {
    final List<Sample> samples = new ArrayList<>();
    final int stepsPerTarget = (int) Math.round(kSecondsPerTarget / Robot.periodSecs);

    // The arm starts resting on its lower stop (-90°). Phoenix reports 0 until it has seen the
    // sim state, and can take a moment to start up, so run disabled for at least 2 s and until
    // it reports the arm at rest before enabling
    for (int i = 0; i < kMaxSettleSeconds / Robot.periodSecs; i++) {
      step();
      if (i >= 2.0 / Robot.periodSecs && m_arm.getAngleDegrees() < -85.0) {
        break;
      }
    }
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    final double start = m_nextStepNanos / 1e9;
    if (!tunablesSetting().isEmpty()) {
      System.out.println("Tunables: " + tunablesSetting());
    }
    System.out.printf("Resting at %.1f deg%n", m_arm.getAngleDegrees());
    for (double target : kTargets) {
      CommandScheduler.getInstance().schedule(m_arm.setAngleCommand(target));
      for (int i = 0; i < stepsPerTarget; i++) {
        Unmanaged.feedEnable(100);
        step();
        samples.add(
            new Sample(
                m_nextStepNanos / 1e9 - start, target, m_arm.getAngleDegrees(), m_profileAngle.get()));
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
    double previousTarget = samples.get(0).angle();

    System.out.printf("%n%-18s %10s %13s %15s%n", "Move", "Overshoot", "Last 2s p-p", "Last 2s err");
    for (int seg = 0; seg < samples.size() / stepsPerTarget; seg++) {
      final List<Sample> segment =
          samples.subList(seg * stepsPerTarget, (seg + 1) * stepsPerTarget);
      final double target = segment.get(0).target();
      final double direction = Math.signum(target - segment.get(0).angle());

      double overshoot = 0.0;
      for (Sample s : segment) {
        overshoot = Math.max(overshoot, direction * (s.angle() - target));
      }

      double min = Double.MAX_VALUE;
      double max = -Double.MAX_VALUE;
      double errorSum = 0.0;
      for (Sample s : segment.subList(segment.size() - settleWindow, segment.size())) {
        min = Math.min(min, s.angle());
        max = Math.max(max, s.angle());
        errorSum += s.angle() - target;
      }

      System.out.printf("%-18s %8.1f°  %12.1f°  %+13.1f°%n",
          String.format("%.0f° -> %.0f°", previousTarget, target),
          overshoot, max - min, errorSum / settleWindow);
      previousTarget = target;
    }
  }

  private static void writeCsv(List<Sample> samples) throws IOException {
    final Path dir = Path.of("build", "response");
    Files.createDirectories(dir);
    final String tunables = tunablesSetting().replace(',', '_').replace('/', '-');
    final Path file =
        dir.resolve(checkpointName() + (tunables.isEmpty() ? "" : "_" + tunables) + ".csv");
    try (PrintWriter out = new PrintWriter(Files.newBufferedWriter(file))) {
      out.println("time_s,target_deg,angle_deg,profile_deg");
      for (Sample s : samples) {
        out.printf("%.3f,%.2f,%.3f,%.3f%n", s.time(), s.target(), s.angle(), s.profile());
      }
    }
    System.out.println("Wrote " + file.toAbsolutePath());
  }

  /** Names the CSV after the checked-out checkpoint tag, e.g. "2027-arm" or "...-dirty". */
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
