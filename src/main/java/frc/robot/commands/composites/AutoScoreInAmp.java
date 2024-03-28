package frc.robot.commands.composites;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AutoGoToAprilTag;
import frc.robot.commands.OutputTimed;
import frc.robot.commands.SetArmPositionCommand;
import frc.robot.commands.WaitUntilArmDown;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoScoreInAmp extends SequentialCommandGroup {
    

    
    public AutoScoreInAmp(
        SwerveSubsystem swerveSubsystem,
        VisionSubsystem visionSubsystem,
        ArmSubsystem armSubsystem,
        IntakeSubsystem intakeSubsystem
    ) {
        
        addCommands(
            new WaitUntilArmDown(armSubsystem),

            // Go to the april tag
            new AutoGoToAprilTag(
                swerveSubsystem,
                visionSubsystem,
                VisionConstants.kAmpXGoal,
                VisionConstants.kAmpYGoal,
                VisionConstants.kAmpRotGoal
            ),


            // Move the arm up
            //new SetArmPositionCommand(armSubsystem, ArmConstants.kArmScoringPosition),
            new SetArmPositionCommand(armSubsystem, ArmConstants.kArmScoringPosition),


            // Wait until the arm has moved to the right position
            new WaitUntilCommand(new BooleanSupplier() {
                @Override
                public boolean getAsBoolean() {
                    return armSubsystem.getCurrentPosition() >= 0.23;
                }
            }),

            // Run the intake
            new OutputTimed(intakeSubsystem, 1.0),

            // Move the arm back to middle
            new SetArmPositionCommand(armSubsystem, ArmConstants.kArmDrivingPosition)
        );
    }
}
