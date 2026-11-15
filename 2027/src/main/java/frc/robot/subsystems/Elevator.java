package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import dev.doglog.DogLog;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.Units;
import org.wpilib.system.RobotController;
import org.wpilib.simulation.ElevatorSim;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.smartdashboard.Mechanism2d;
import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.smartdashboard.MechanismRoot2d;
import org.wpilib.util.Color;
import org.wpilib.util.Color8Bit;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import frc.robot.Robot;

public class Elevator extends SubsystemBase {
  private final double kGearRatio = 3.0;
  private final double kDrumDiameter = Units.inchesToMeters(1.0);
  private final double kRotationsPerMeter = kGearRatio / (Math.PI * kDrumDiameter);

  private final TalonFX m_motor = new TalonFX(0, new CANBus());

  private ControlMode m_controlMode = ControlMode.VOLTAGE;
  private double m_targetVoltage = 0.0;
  private double m_targetPosition = 0.0;

  // The TalonFX runs the profile and PIDF itself (Motion Magic); we just send it the target
  private final MotionMagicVoltage m_magic = new MotionMagicVoltage(0.0);
  private final TalonFXConfiguration m_config = new TalonFXConfiguration();

  // Tunable in motor rotations, under /Tunables/TalonFX/ (so old kP = 100 V/m values can't
  // land here). Changing one pushes the new settings to the TalonFX right away.
  private final TunableDouble m_tunable_kP =
      DogLog.tunable("TalonFX/kP", 2.66, v -> { m_config.Slot0.kP = v; applyGains(); });
  private final TunableDouble m_tunable_kI =
      DogLog.tunable("TalonFX/kI", 0.0, v -> { m_config.Slot0.kI = v; applyGains(); });
  private final TunableDouble m_tunable_kD =
      DogLog.tunable("TalonFX/kD", 0.0, v -> { m_config.Slot0.kD = v; applyGains(); });
  private final TunableDouble m_tunable_kG =
      DogLog.tunable("TalonFX/kG", 1.1, v -> { m_config.Slot0.kG = v; applyGains(); });
  private final TunableDouble m_tunable_kV =
      DogLog.tunable("TalonFX/kV", 0.122, v -> { m_config.Slot0.kV = v; applyGains(); });
  private final TunableDouble m_tunable_cruiseVelocity =
      DogLog.tunable("TalonFX/CruiseVelocity", 37.6, v -> {
        m_config.MotionMagic.MotionMagicCruiseVelocity = v;
        applyProfileLimits();
      });
  private final TunableDouble m_tunable_acceleration =
      DogLog.tunable("TalonFX/Acceleration", 75.2, v -> {
        m_config.MotionMagic.MotionMagicAcceleration = v;
        applyProfileLimits();
      });

  enum ControlMode {
    VOLTAGE,
    POSITION,
  }

  public Elevator() {
    // Start from the tunables' values (motor rotations: our tuned values / 37.6 per meter)
    m_config.Slot0.kP = m_tunable_kP.get(); // V per rotation of error (100 V/m)
    m_config.Slot0.kI = m_tunable_kI.get();
    m_config.Slot0.kD = m_tunable_kD.get();
    m_config.Slot0.kG = m_tunable_kG.get(); // V to hold against gravity
    m_config.Slot0.kV = m_tunable_kV.get(); // V per rotation/s (4.6 V per m/s)
    m_config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    m_config.MotionMagic.MotionMagicCruiseVelocity = m_tunable_cruiseVelocity.get(); // rot/s
    m_config.MotionMagic.MotionMagicAcceleration = m_tunable_acceleration.get(); // rot/s²
    m_motor.getConfigurator().apply(m_config);
  }

  // Sends just the changed group of settings to the TalonFX
  private void applyGains() {
    m_motor.getConfigurator().apply(m_config.Slot0);
    System.out.printf(
        "TalonFX gains: kP=%.3f kI=%.3f kD=%.3f kG=%.3f kV=%.3f%n",
        m_config.Slot0.kP, m_config.Slot0.kI, m_config.Slot0.kD,
        m_config.Slot0.kG, m_config.Slot0.kV);
  }

  private void applyProfileLimits() {
    m_motor.getConfigurator().apply(m_config.MotionMagic);
    System.out.printf(
        "TalonFX profile: cruise %.1f rot/s, acceleration %.1f rot/s²%n",
        m_config.MotionMagic.MotionMagicCruiseVelocity,
        m_config.MotionMagic.MotionMagicAcceleration);
  }

  public Command setVoltageCommand(double voltage) {
    return runOnce(
        () -> {
          m_controlMode = ControlMode.VOLTAGE;
          m_targetVoltage = voltage;
        });
  }

  public Command setPositionCommand(double setpoint) {
    return runOnce(
        () -> {
          m_controlMode = ControlMode.POSITION;
          m_targetPosition = setpoint;
        });
  }

  public double getCurrentPosition() {
    return m_motor.getPosition().getValueAsDouble() / kRotationsPerMeter;
  }

  public double getCurrentVelocity() {
    return m_motor.getVelocity().getValueAsDouble() / kRotationsPerMeter;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    final double currentPosition = getCurrentPosition();

    switch (m_controlMode) {
      case VOLTAGE:
        {
          m_motor.setVoltage(m_targetVoltage);
          break;
        }
      case POSITION:
        {
          m_motor.setControl(m_magic.withPosition(m_targetPosition * kRotationsPerMeter));

          // The TalonFX's own profile position, for graphing
          DogLog.log(
              "Elevator/Profile Position (m)",
              m_motor.getClosedLoopReference().getValueAsDouble() / kRotationsPerMeter);
          break;
        }
    }

    DogLog.log("Elevator/Control Mode", m_controlMode);
    DogLog.log("Elevator/Duty Cycle", m_motor.getDutyCycle().getValueAsDouble());
    DogLog.log("Elevator/Current Position (m)", currentPosition);
    DogLog.log("Elevator/Target Position (m)", m_targetPosition);
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          DCMotor.getKrakenX60Foc(1), kGearRatio, 20.0, 0.5 * kDrumDiameter, 0.0, 2.0, true, 1.0);

  // Visualization
  private final double kBottomOffset = 20.0;
  private final double kPixelsPerMeter = 100.0;
  private final Mechanism2d m_simViz = new Mechanism2d(200, 300);
  private final MechanismRoot2d m_zRoot = m_simViz.getRoot("Z Root", 100, kBottomOffset);
  // The rail is thinner than the carriage so the carriage stays visible no matter which
  // root the dashboard draws on top (the SimGUI and AdvantageScope use different orders).
  private final MechanismLigament2d m_z =
      m_zRoot.append(new MechanismLigament2d("Z", 220, 90, 4, new Color8Bit(Color.GRAY)));
  private final MechanismRoot2d m_carriageRoot = m_simViz.getRoot("Carriage", 0, 0);
  private final MechanismLigament2d m_carriage =
      m_carriageRoot.append(
          new MechanismLigament2d("Carriage", 20, 90, 14, new Color8Bit(Color.YELLOW)));

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

    final TalonFXSimState motorSim = m_motor.getSimState();
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_elevatorSim.setInputVoltage(m_motor.getMotorVoltage().getValueAsDouble());
    m_elevatorSim.update(Robot.periodSecs);

    // Convert motor rotations and motor rotations per second
    motorSim.setRawRotorPosition(m_elevatorSim.getPosition() * kRotationsPerMeter);
    motorSim.setRotorVelocity(m_elevatorSim.getVelocity() * kRotationsPerMeter);

    // Update viz
    m_carriageRoot.setPosition(
        100, kBottomOffset + m_elevatorSim.getPosition() * kPixelsPerMeter);

    // Telemetry replaces SmartDashboard.putData() in 2027: it must be logged every loop.
    // View it in the dashboard under NetworkTables -> Telemetry -> Elevator Viz.
    Telemetry.log("Elevator Viz", m_simViz);
  }
  // --- END STUFF FOR SIMULATION ---
}
