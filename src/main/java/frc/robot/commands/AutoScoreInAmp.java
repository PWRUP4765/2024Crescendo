package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.VisionConstants;
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
            // Go to the april tag
            new AutoGoToAprilTag(
                swerveSubsystem,
                visionSubsystem,
                VisionConstants.kAmpXGoal,
                VisionConstants.kAmpYGoal,
                VisionConstants.kAmpRotGoal
            ),

            // Move the arm up
            armSubsystem.runOnce(() -> armSubsystem.setPosition(ArmConstants.kArmScoringPosition)),
            
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
            armSubsystem.runOnce(() -> armSubsystem.setPosition(ArmConstants.kArmDrivingPosition))
        );
    }
}
