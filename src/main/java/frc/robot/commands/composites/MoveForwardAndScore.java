package frc.robot.commands.composites;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveUntilSeeAprilTag;
import frc.robot.commands.WaitUntilArmDown;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class MoveForwardAndScore extends SequentialCommandGroup {


    public MoveForwardAndScore(
        SwerveSubsystem swerveSubsystem,
        VisionSubsystem visionSubsystem,
        ArmSubsystem armSubsystem,
        IntakeSubsystem intakeSubsystem
    ) {


        addCommands(
            new WaitUntilArmDown(armSubsystem),
            new MoveUntilSeeAprilTag(
                swerveSubsystem,
                visionSubsystem,
                0.0,
                0.2
            ),
            new AutoScoreInAmp(
                swerveSubsystem,
                visionSubsystem,
                armSubsystem,
                intakeSubsystem
            )
        );
    }
}
