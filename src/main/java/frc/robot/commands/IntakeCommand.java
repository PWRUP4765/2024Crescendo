// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

/** An example command that uses an example subsystem. */
public class IntakeCommand extends Command {

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final IntakeSubsystem m_IntakeSubsystem;

  private final int IntakeSpeed = (int) IntakeConstants.kIntakeSpeed;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public IntakeCommand(IntakeSubsystem IntakeSubsystem) {
    this.m_IntakeSubsystem = IntakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(IntakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_IntakeSubsystem.setMotor(IntakeSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.setMotor(0);
  }

  @Override
  public boolean isFinished() {
    return m_IntakeSubsystem.isDetected();
  }
}
