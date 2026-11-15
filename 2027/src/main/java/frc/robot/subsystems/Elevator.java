package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import dev.doglog.DogLog;
import org.wpilib.math.controller.ElevatorFeedforward;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.trajectory.TrapezoidProfile.Constraints;
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

  private TunableDouble m_tunable_kP = DogLog.tunable("kP", 100.0);
  private TunableDouble m_tunable_kD = DogLog.tunable("kD", 0.0);
  private TunableDouble m_tunable_kI = DogLog.tunable("kI", 0.0);
  private TunableDouble m_tunable_kIZone = DogLog.tunable("kIZone", 0.01);

  // WPILib's versions of what we built by hand: PID following a trapezoid profile, and
  // gravity + velocity feedforward (kS = 0, kG = 1.1 V, kV = 4.6 V per m/s)
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(100.0, 0.0, 0.0, new Constraints(1.0, 2.0), Robot.periodSecs);
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(0.0, 1.1, 4.6);

  enum ControlMode {
    VOLTAGE,
    POSITION,
  }

  public Elevator() {}

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

          // Start the profile from where the elevator is now (also clears the I term)
          m_controller.reset(getCurrentPosition(), getCurrentVelocity());
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
          m_controller.setPID(m_tunable_kP.get(), m_tunable_kI.get(), m_tunable_kD.get());
          m_controller.setIZone(m_tunable_kIZone.get());

          // PID toward the profile's next state, plus feedforward for that state's velocity
          double pidVolts = m_controller.calculate(currentPosition, m_targetPosition);
          var setpoint = m_controller.getSetpoint();
          DogLog.log("Elevator/Profile Position (m)", setpoint.position);
          DogLog.log("Elevator/Profile Velocity (m per s)", setpoint.velocity);

          m_motor.setVoltage(pidVolts + m_feedforward.calculate(setpoint.velocity));
          break;
        }
    }

    DogLog.log("Elevator/Control Mode", m_controlMode);
    DogLog.log("Elevator/Duty Cycle", m_motor.getDutyCycle().getValueAsDouble());
    DogLog.log("Elevator/Current Position (m)", currentPosition);
    DogLog.log("Elevator/Target Position (m)", m_targetPosition);
    DogLog.log("Elevator/iError", m_controller.getAccumulatedError());
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
