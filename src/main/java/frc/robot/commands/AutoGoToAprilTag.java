package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.hardware.HardwareComponents;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionTarget;
import frc.robot.util.DriverAllianceUtil;

public class AutoGoToAprilTag extends Command {

    VisionSubsystem m_vision;
    SwerveSubsystem m_swerveSubsystem;

    double m_desiredX, m_desiredY, m_desiredRot;

    VisionTarget m_lastTarget;
    int timesSinceLastSight = 100;

    public AutoGoToAprilTag(
        SwerveSubsystem swerveSubsystem,
        VisionSubsystem vision,
        double x,
        double y,
        double rot
    ) {
        m_vision = vision;
        m_swerveSubsystem = swerveSubsystem;

        m_desiredX = x;
        m_desiredY = y;
        m_desiredRot = rot;

        addRequirements(swerveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_lastTarget = null;
        timesSinceLastSight = 100;
    }

    @Override
    public void execute() {
        VisionTarget target = m_vision.findBestTarget();

        // If the camera hasn't seen the target recently, uses the previous input's data
        if (target == null && timesSinceLastSight < 4) {
            target = m_lastTarget;
            timesSinceLastSight++;
        }


        if (target != null) {
            m_vision.updateCalculations(target, m_desiredX, m_desiredY, m_desiredRot);

            double sidewaysSpeedX = m_vision.getSidewaysSpeedX();
            double forwardSpeedY = m_vision.getForwardSpeedY();
            double rotationSpeed = m_vision.getRotationSpeed();

            m_swerveSubsystem.drive(sidewaysSpeedX, forwardSpeedY, rotationSpeed, SwerveConstants.kAutonSpeedMultiplier);
        }
    }

    @Override
    public void end(boolean interrupted) {
        HardwareComponents.gyro.setCurrentAngle(DriverAllianceUtil.getCurrentAlliance() == DriverStation.Alliance.Blue ? -0.25 : 0.25);
        m_swerveSubsystem.drive(0,0,0, SwerveConstants.kAutonSpeedMultiplier);
    }

    @Override
    public boolean isFinished() {
        return m_vision.isAtTarget();
    }
}
