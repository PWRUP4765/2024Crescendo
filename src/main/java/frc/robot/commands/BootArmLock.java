package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbArmSubsystem;

public class BootArmLock extends Command {
    
    public BootArmLock(ArmSubsystem armSubsystem, ClimbArmSubsystem climbArmSubsystem) {
        if (climbArmSubsystem.isArmOnBottom()) {
            armSubsystem.setLocked(true, 0);
        }
    }
}
