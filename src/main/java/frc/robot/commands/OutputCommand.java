// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// WORK IN PROGRESS
public class OutputCommand extends Command {

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final IntakeSubsystem m_intakeSubsystem;
  private final ArmSubsystem m_armSubsystem;

  private final double outputSpeed = IntakeConstants.kOutputSpeed;

  /**
   * @param intakeSubsystem The subsystem used by this command.
   */
  public OutputCommand(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
    this.m_intakeSubsystem = intakeSubsystem;
    this.m_armSubsystem = armSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // setting the motor speed to outputSpeed
    if (m_armSubsystem.getCurrentPosition() > 0.23) {
    m_intakeSubsystem.setMotor(outputSpeed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.setMotor(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
