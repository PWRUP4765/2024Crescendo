package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.DriverAllianceUtil;

public class SnapToAmp extends Command {
    
    public final SwerveSubsystem m_swerveSubsystem;

    public SnapToAmp(SwerveSubsystem swerveSubsystem) {
        m_swerveSubsystem = swerveSubsystem;
    }

    @Override
    public void initialize() {
        m_swerveSubsystem.setDesiredDirection(
            DriverAllianceUtil.getCurrentAlliance() == DriverStation.Alliance.Blue ? -0.25 : 0.25
        );
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
