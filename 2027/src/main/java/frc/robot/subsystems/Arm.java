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
import org.wpilib.simulation.SingleJointedArmSim;
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

public class Arm extends SubsystemBase {
  private final double kGearRatio = 40.0;
  private final double kArmLength = 0.7; // meters, pivot to tip
  private final double kArmMass = 5.0; // kg

  private final TalonFX m_motor = new TalonFX(1, new CANBus());

  private ControlMode m_controlMode = ControlMode.VOLTAGE;
  private double m_targetVoltage = 0.0;
  private double m_targetAngle = -90.0; // degrees; 0 = horizontal, 90 = straight up

  // The TalonFX runs the profile and PIDF itself (Motion Magic); we just send it the target
  private final MotionMagicVoltage m_magic = new MotionMagicVoltage(0.0);
  private final TalonFXConfiguration m_config = new TalonFXConfiguration();

  // Tunable in arm rotations (SensorToMechanismRatio = gear ratio), under /Tunables/Arm/.
  // Changing one pushes the new settings to the TalonFX right away.
  private final TunableDouble m_tunable_kP =
      DogLog.tunable("Arm/kP", 100.0, v -> { m_config.Slot0.kP = v; applyGains(); });
  private final TunableDouble m_tunable_kI =
      DogLog.tunable("Arm/kI", 0.0, v -> { m_config.Slot0.kI = v; applyGains(); });
  private final TunableDouble m_tunable_kD =
      DogLog.tunable("Arm/kD", 0.0, v -> { m_config.Slot0.kD = v; applyGains(); });
  private final TunableDouble m_tunable_kG =
      DogLog.tunable("Arm/kG", 0.55, v -> { m_config.Slot0.kG = v; applyGains(); });
  private final TunableDouble m_tunable_kV =
      DogLog.tunable("Arm/kV", 4.98, v -> { m_config.Slot0.kV = v; applyGains(); });
  private final TunableDouble m_tunable_cruiseVelocity =
      DogLog.tunable("Arm/CruiseVelocity", 0.5, v -> {
        m_config.MotionMagic.MotionMagicCruiseVelocity = v;
        applyProfileLimits();
      });
  private final TunableDouble m_tunable_acceleration =
      DogLog.tunable("Arm/Acceleration", 1.0, v -> {
        m_config.MotionMagic.MotionMagicAcceleration = v;
        applyProfileLimits();
      });

  enum ControlMode {
    VOLTAGE,
    POSITION,
  }

  public Arm() {
    // Measure in arm rotations instead of motor rotations, with 0 = horizontal
    m_config.Feedback.SensorToMechanismRatio = kGearRatio;
    m_config.Slot0.kP = m_tunable_kP.get(); // V per arm rotation of error
    m_config.Slot0.kI = m_tunable_kI.get();
    m_config.Slot0.kD = m_tunable_kD.get();
    m_config.Slot0.kG = m_tunable_kG.get(); // V to hold the arm horizontal
    m_config.Slot0.kV = m_tunable_kV.get(); // V per arm rotation/s (12 V / 2.41 rot/s)
    // The same constant gravity feedforward we used for the elevator
    m_config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    m_config.MotionMagic.MotionMagicCruiseVelocity = m_tunable_cruiseVelocity.get(); // rot/s
    m_config.MotionMagic.MotionMagicAcceleration = m_tunable_acceleration.get(); // rot/s²
    m_motor.getConfigurator().apply(m_config);
  }

  // Sends just the changed group of settings to the TalonFX
  private void applyGains() {
    m_motor.getConfigurator().apply(m_config.Slot0);
    System.out.printf(
        "Arm gains: kP=%.3f kI=%.3f kD=%.3f kG=%.3f kV=%.3f%n",
        m_config.Slot0.kP, m_config.Slot0.kI, m_config.Slot0.kD,
        m_config.Slot0.kG, m_config.Slot0.kV);
  }

  private void applyProfileLimits() {
    m_motor.getConfigurator().apply(m_config.MotionMagic);
    System.out.printf(
        "Arm profile: cruise %.2f rot/s, acceleration %.2f rot/s²%n",
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

  public Command setAngleCommand(double degrees) {
    return runOnce(
        () -> {
          m_controlMode = ControlMode.POSITION;
          m_targetAngle = degrees;
        });
  }

  public double getAngleDegrees() {
    return m_motor.getPosition().getValueAsDouble() * 360.0;
  }

  @Override
  public void periodic() {
    switch (m_controlMode) {
      case VOLTAGE:
        {
          m_motor.setVoltage(m_targetVoltage);
          break;
        }
      case POSITION:
        {
          m_motor.setControl(m_magic.withPosition(m_targetAngle / 360.0));

          // The TalonFX's own profile angle, for graphing
          DogLog.log(
              "Arm/Profile Angle (deg)",
              m_motor.getClosedLoopReference().getValueAsDouble() * 360.0);
          break;
        }
    }

    DogLog.log("Arm/Control Mode", m_controlMode);
    DogLog.log("Arm/Applied Voltage", m_motor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Arm/Current Angle (deg)", getAngleDegrees());
    DogLog.log("Arm/Target Angle (deg)", m_targetAngle);
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          kGearRatio,
          SingleJointedArmSim.estimateMOI(kArmLength, kArmMass),
          kArmLength,
          Units.degreesToRadians(-90.0), // hard stop: hanging straight down
          Units.degreesToRadians(180.0), // hard stop: folded over the top
          true,
          Units.degreesToRadians(-90.0)); // starts resting on the lower stop

  // Visualization: a tower with the arm pivoting at the top, plus a thin line at the target
  private final double kPixelsPerMeter = 200.0;
  private final Mechanism2d m_simViz = new Mechanism2d(400, 400);
  private final MechanismRoot2d m_baseRoot = m_simViz.getRoot("Base", 200, 20);
  private final MechanismLigament2d m_tower =
      m_baseRoot.append(new MechanismLigament2d("Tower", 180, 90, 10, new Color8Bit(Color.GRAY)));
  private final MechanismRoot2d m_targetRoot = m_simViz.getRoot("Target", 200, 200);
  private final MechanismLigament2d m_target =
      m_targetRoot.append(
          new MechanismLigament2d(
              "Target", kArmLength * kPixelsPerMeter, -90, 2, new Color8Bit(Color.LIGHT_GRAY)));
  private final MechanismRoot2d m_armRoot = m_simViz.getRoot("Pivot", 200, 200);
  private final MechanismLigament2d m_arm =
      m_armRoot.append(
          new MechanismLigament2d(
              "Arm", kArmLength * kPixelsPerMeter, -90, 10, new Color8Bit(Color.YELLOW)));

  @Override
  public void simulationPeriodic() {
    final TalonFXSimState motorSim = m_motor.getSimState();
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_armSim.setInputVoltage(m_motor.getMotorVoltage().getValueAsDouble());
    m_armSim.update(Robot.periodSecs);

    // Arm radians -> motor rotations (and rad/s -> motor rotations/s)
    motorSim.setRawRotorPosition(m_armSim.getAngle() / (2 * Math.PI) * kGearRatio);
    motorSim.setRotorVelocity(m_armSim.getVelocity() / (2 * Math.PI) * kGearRatio);

    m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngle()));
    m_target.setAngle(m_targetAngle);
    // View it in the dashboard under NetworkTables -> Telemetry -> Arm Viz.
    Telemetry.log("Arm Viz", m_simViz);
  }
  // --- END STUFF FOR SIMULATION ---
}
