package frc.robot.commands.finals;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveDirectionTimed;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.DriverAllianceUtil;

public class LeaveBox extends SequentialCommandGroup {
    public LeaveBox(SwerveSubsystem swerveSubsystem) {
        addCommands(
            new MoveDirectionTimed(swerveSubsystem, 0, 0.4, 2500)
        );
    }
}
