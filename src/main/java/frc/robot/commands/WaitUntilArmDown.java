package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class WaitUntilArmDown extends Command {
    
    private final ArmSubsystem m_armSubsystem;

    public WaitUntilArmDown(ArmSubsystem armSubsystem) {
        m_armSubsystem = armSubsystem;
    }

    @Override
    public boolean isFinished() {
        return m_armSubsystem.getCurrentPosition() <= 0.14;
    }
}
