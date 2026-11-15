// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.driverstation.Gamepad.Axis;
import frc.robot.subsystems.Arm;

public class RobotContainer {
  // Controllers
  private static final CommandGamepad m_driverController = new CommandGamepad(0);

  // Subsystems
  private static final Arm m_arm = new Arm();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.faceDown().whileTrue(m_arm.setVoltageCommand(-4.0)); // keyboard 0: Z
    m_driverController.faceDown().onFalse(m_arm.setVoltageCommand(0.0));
    m_driverController.faceRight().whileTrue(m_arm.setVoltageCommand(4.0)); // keyboard 0: X
    m_driverController.faceRight().onFalse(m_arm.setVoltageCommand(0.0));

    m_driverController.faceLeft().onTrue(m_arm.setAngleCommand(0.0)); // keyboard 0: C (horizontal)
    m_driverController.faceUp().onTrue(m_arm.setAngleCommand(90.0)); // keyboard 0: V (straight up)
    // Keyboard 0 has only four buttons, so W/S (the left stick's Y axis) pick the 45° targets
    m_driverController.axisLessThan(Axis.LEFT_Y, -0.5).onTrue(m_arm.setAngleCommand(45.0)); // W
    m_driverController.axisGreaterThan(Axis.LEFT_Y, 0.5).onTrue(m_arm.setAngleCommand(-45.0)); // S
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
