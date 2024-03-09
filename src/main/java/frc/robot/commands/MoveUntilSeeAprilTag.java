package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class MoveUntilSeeAprilTag extends Command {
    
    private final SwerveSubsystem m_swerveSubsystem;
    private final VisionSubsystem m_visionSubsystem;

    private final double m_xSpeed;
    private final double m_ySpeed;

    private long time;

    public MoveUntilSeeAprilTag(
        SwerveSubsystem swerveSubsystem,
        VisionSubsystem visionSubsystem,
        double xSpeed,
        double ySpeed
    ) {
        this.m_swerveSubsystem = swerveSubsystem;
        this.m_visionSubsystem = visionSubsystem;
        this.m_xSpeed = xSpeed;
        this.m_ySpeed = ySpeed;
    }

    @Override
    public void initialize() {
        time = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        m_swerveSubsystem.drive(m_xSpeed, m_ySpeed, 0, SwerveConstants.kAutonSpeedMultiplier);
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveSubsystem.drive(0, 0, 0, SwerveConstants.kAutonSpeedMultiplier);
    }

    @Override
    public boolean isFinished() {
        return (m_visionSubsystem.findBestTarget() != null) || System.currentTimeMillis() - time >= 10000;
    }
}
