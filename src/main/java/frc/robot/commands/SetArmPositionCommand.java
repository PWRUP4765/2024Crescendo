package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class SetArmPositionCommand extends Command {

    private final ArmSubsystem m_armSubsystem;
    private final double setPosition;

    public SetArmPositionCommand(ArmSubsystem armSubsystem, double position) {

        m_armSubsystem = armSubsystem;
        setPosition = position;
    }

    @Override
    public void initialize() {
        m_armSubsystem.setPosition(setPosition);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
