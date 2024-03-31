package frc.robot.commands.finals;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.DriverAllianceUtil;
import frc.robot.commands.MoveDirectionTimed;
import frc.robot.commands.TurnAmount;
import frc.robot.commands.composites.AutoScoreInAmp;
import frc.robot.commands.composites.MoveAwayFromAmp;
import frc.robot.commands.composites.MoveForwardAndScore;
import frc.robot.commands.MoveForwardIntake;
import frc.robot.commands.MoveUntilSeeAprilTag;

public class DoubleScore extends SequentialCommandGroup{
    
    public DoubleScore(
        SwerveSubsystem swerveSubsystem,
        VisionSubsystem visionSubsystem,
        ArmSubsystem armSubsystem,
        IntakeSubsystem intakeSubsystem
    ) {

        addCommands(
            new MoveForwardAndScore(
                swerveSubsystem,
                visionSubsystem,
                armSubsystem,
                intakeSubsystem
            ),
            new MoveDirectionTimed(swerveSubsystem, 0, -0.1, 500),
            new TurnAmount(swerveSubsystem, DriverAllianceUtil.getCurrentAlliance() == DriverStation.Alliance.Blue ? -0.208 : 0.208),
            new MoveForwardIntake(
                swerveSubsystem,
                armSubsystem,
                intakeSubsystem
            ),
            new MoveDirectionTimed(swerveSubsystem, 0, 0.2, 1000),
            new TurnAmount(swerveSubsystem, DriverAllianceUtil.getCurrentAlliance() == DriverStation.Alliance.Blue ? 0.208 : -0.208),
            new MoveUntilSeeAprilTag(
                swerveSubsystem,
                visionSubsystem,
                DriverAllianceUtil.getCurrentAlliance() == DriverStation.Alliance.Blue ? -0.2 : 0.2,
                0.05
            ),
            new AutoScoreInAmp(
                swerveSubsystem,
                visionSubsystem,
                armSubsystem,
                intakeSubsystem
            ),
            new MoveAwayFromAmp(swerveSubsystem)
        );
    }
}
