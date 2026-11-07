// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandGamepad;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
  // Controllers
  private static final CommandGamepad m_driverController = new CommandGamepad(0);

  // Subsystems
  private static final Elevator m_elevator = new Elevator();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.faceDown().whileTrue(m_elevator.setVoltageCommand(-12.0)); // keyboard 0: Z
    m_driverController.faceDown().onFalse(m_elevator.setVoltageCommand(0.0));
    m_driverController.faceRight().whileTrue(m_elevator.setVoltageCommand(12.0)); // keyboard 0: X
    m_driverController.faceRight().onFalse(m_elevator.setVoltageCommand(0.0));

    m_driverController.faceLeft().onTrue(m_elevator.setPositionCommand(0.5)); // keyboard 0: C
    m_driverController.faceUp().onTrue(m_elevator.setPositionCommand(1.5)); // keyboard 0: V
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
