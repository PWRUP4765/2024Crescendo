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
  private final SwerveSubsystem m_swerveSubsystem;

  private final double outputSpeed = IntakeConstants.kOutputSpeed;

  /**
   * @param intakeSubsystem The subsystem used by this command.
   */
  public OutputCommand(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem, SwerveSubsystem swerveSubsystem) {
    this.m_intakeSubsystem = intakeSubsystem;
    this.m_armSubsystem = armSubsystem;
    this.m_swerveSubsystem = swerveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    m_armSubsystem.setPosition(ArmConstants.kArmScoringPosition);
    m_swerveSubsystem.setSpeedMultiplier(SwerveConstants.kIntakeSpeedMultiplier);
  }

  @Override
  public void execute() {
    m_intakeSubsystem.setMotor(outputSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.setMotor(0);
    m_armSubsystem.setPosition(ArmConstants.kArmDrivingPosition);
    m_swerveSubsystem.setSpeedMultiplier(SwerveConstants.kDefaultSpeedMultiplier);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
