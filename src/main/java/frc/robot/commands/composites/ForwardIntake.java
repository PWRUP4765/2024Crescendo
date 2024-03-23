package frc.robot.commands.composites;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.MoveForwardIntake;
import frc.robot.commands.SetArmPositionCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ForwardIntake extends SequentialCommandGroup {
    
    public ForwardIntake(
        SwerveSubsystem swerveSubsystem,
        ArmSubsystem armSubsystem,
        IntakeSubsystem intakeSubsystem
    ) {

        addCommands(
            new SetArmPositionCommand(armSubsystem, ArmConstants.kArmFlatPosition),
            new WaitUntilCommand(new BooleanSupplier() {
                @Override
                public boolean getAsBoolean() {
                    return armSubsystem.getCurrentPosition() <= 0.04;
                }
            }),
            new MoveForwardIntake(swerveSubsystem, armSubsystem, intakeSubsystem)
        );
    }
}
