package frc.robot.commands.finals;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.MoveDirectionTimed;
import frc.robot.commands.TurnAmount;
import frc.robot.commands.composites.MoveForwardAndScore;
import frc.robot.commands.MoveForwardIntake;

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
            new TurnAmount(swerveSubsystem, -0.05),
            new MoveForwardIntake(
                swerveSubsystem,
                armSubsystem,
                intakeSubsystem
            ),
            new MoveForwardAndScore(
                swerveSubsystem,
                visionSubsystem,
                armSubsystem,
                intakeSubsystem
            )

        );
    }
}
