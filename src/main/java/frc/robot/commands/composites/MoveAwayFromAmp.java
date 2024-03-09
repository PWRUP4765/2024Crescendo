package frc.robot.commands.composites;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveDirectionTimed;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.DriverAllianceUtil;

public class MoveAwayFromAmp extends SequentialCommandGroup {
    public MoveAwayFromAmp(
        SwerveSubsystem swerveSubsystem
    ) {


        addCommands(
            new MoveDirectionTimed(swerveSubsystem, 0, -0.1, 500),

            new MoveDirectionTimed(
                swerveSubsystem,
                DriverAllianceUtil.getCurrentAlliance() == DriverStation.Alliance.Blue ? 0.2 : -0.2,
                0,
                4000
            )
        );
    }
}
