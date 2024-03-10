package frc.robot.commands.finals;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TurnAmount;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.DriverAllianceUtil;

public class MoveScoreRetreatTurn extends SequentialCommandGroup {
    
    public MoveScoreRetreatTurn(
        SwerveSubsystem swerveSubsystem,
        VisionSubsystem visionSubsystem,
        ArmSubsystem armSubsystem,
        IntakeSubsystem intakeSubsystem
    ) {

        addCommands(
            new MoveScoreRetreat(swerveSubsystem, visionSubsystem, armSubsystem, intakeSubsystem),
            new TurnAmount(
                swerveSubsystem,
                DriverAllianceUtil.getCurrentAlliance() == DriverStation.Alliance.Blue ? 0.125 : -0.125
            )
        );
    }
}
