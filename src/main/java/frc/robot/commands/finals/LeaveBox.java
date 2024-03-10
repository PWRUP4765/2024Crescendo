package frc.robot.commands.finals;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveDirectionTimed;
import frc.robot.subsystems.SwerveSubsystem;

public class LeaveBox extends SequentialCommandGroup {
    public LeaveBox(SwerveSubsystem swerveSubsystem) {
        addCommands(
            new MoveDirectionTimed(swerveSubsystem, 0, 0.4, 2500)
        );
    }
}
