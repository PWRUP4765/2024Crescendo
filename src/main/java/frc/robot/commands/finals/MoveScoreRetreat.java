package frc.robot.commands.finals;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.composites.MoveAwayFromAmp;
import frc.robot.commands.composites.MoveForwardAndScore;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class MoveScoreRetreat extends SequentialCommandGroup {

    public MoveScoreRetreat(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
        this.addCommands(
            new MoveForwardAndScore(swerveSubsystem, visionSubsystem, armSubsystem, intakeSubsystem),
            new MoveAwayFromAmp(swerveSubsystem)
        );
    }
}
