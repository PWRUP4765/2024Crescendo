// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.controller.FlightStick;
import frc.robot.util.controller.LogitechController;

/** An example command that uses an example subsystem. */
public class ScoreAuton extends SequentialCommandGroup {

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ScoreAuton(
    ArmSubsystem armSubsystem,
    IntakeSubsystem intakeSubsystem,
    SwerveSubsystem swerveSubsystem,
    VisionSubsystem visionSubsystem,
    LogitechController m_controller
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new GoToAprilTag(
        m_controller,
        swerveSubsystem,
        visionSubsystem,
        VisionConstants.kAmpXGoal,
        VisionConstants.kAmpYGoal,
        VisionConstants.kAmpRotGoal
      ),
      new OutputPrepCommand(armSubsystem, swerveSubsystem),
      new OutputCommand(intakeSubsystem, armSubsystem)
    );
  }
}
