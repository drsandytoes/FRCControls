package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import dev.doglog.DogLog;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.trajectory.TrapezoidProfile.Constraints;
import org.wpilib.math.trajectory.TrapezoidProfile.State;
import org.wpilib.math.util.Units;
import org.wpilib.system.RobotController;
import org.wpilib.system.Timer;
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
  private TunableDouble m_tunable_kD = DogLog.tunable("kD", 3.0);
  private TunableDouble m_tunable_kI = DogLog.tunable("kI", 200.0);
  private TunableDouble m_tunable_kIZone = DogLog.tunable("kIZone", 0.05);

  private final Timer m_timer = new Timer();
  private double m_lastError = 0.0;
  private double m_iError = 0.0;

  private final Timer m_profileTimer = new Timer();
  private final TrapezoidProfile m_profile = new TrapezoidProfile(new Constraints(1.0, 2.0));
  private State m_startState = new State();
  private State m_goalState = new State();

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

          m_timer.restart();
          m_lastError = 0.0; // the profile starts at the current position
          m_iError = 0.0;

          m_profileTimer.restart();
          m_startState = new State(getCurrentPosition(), getCurrentVelocity());
          m_goalState = new State(m_targetPosition, 0.0);
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
          // Sample profile
          final State profileState =
              m_profile.calculate(m_profileTimer.get(), m_startState, m_goalState);
          DogLog.log("Elevator/Profile Position (m)", profileState.position);
          DogLog.log("Elevator/Profile Velocity (m per s)", profileState.velocity);

          // // PID control w/ trapezoidal profile
          // double error = profileState.position - currentPosition;
          // double dError = (error - m_lastError) / m_timer.get();
          // m_iError = m_iError + error * m_timer.get();

          // // Reset integral if position error is greater than |kIZone|
          // double kIZone = m_tunable_kIZone.get();
          // if (Math.abs(error) > kIZone) {
          //   m_iError = 0.0;
          // }

          // double kP = m_tunable_kP.get();
          // double kD = m_tunable_kD.get();
          // double kI = m_tunable_kI.get();
          // m_motor.setVoltage(error * kP + dError * kD + m_iError * kI);
          // m_timer.reset();
          // m_lastError = error;

          // Velocity FF
          double kV = 4.6; // volts per meter per second
          m_motor.setVoltage(kV * profileState.velocity);
          break;
        }
    }

    DogLog.log("Elevator/Control Mode", m_controlMode);
    DogLog.log("Elevator/Duty Cycle", m_motor.getDutyCycle().getValueAsDouble());
    DogLog.log("Elevator/Current Position (m)", currentPosition);
    DogLog.log("Elevator/Target Position (m)", m_targetPosition);
    DogLog.log("Elevator/iError", m_iError);
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
