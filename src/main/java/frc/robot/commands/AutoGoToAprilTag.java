package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionTarget;

public class AutoGoToAprilTag extends Command {

    VisionSubsystem m_vision;
    SwerveSubsystem m_swerveSubsystem;

    double m_desiredX, m_desiredY, m_desiredRot;

    VisionTarget m_lastTarget;
    int timesSinceLastSight = 100;

    public AutoGoToAprilTag(
        SwerveSubsystem swerveDrive,
        VisionSubsystem vision,
        double x,
        double y,
        double rot
    ) {
        m_vision = vision;
        m_swerveSubsystem = swerveDrive;

        m_desiredX = x;
        m_desiredY = y;
        m_desiredRot = rot;

        addRequirements(swerveDrive);
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
        if (target == null && timesSinceLastSight < 2) {
            target = m_lastTarget;
            timesSinceLastSight++;
        }


        if (target != null) {
            m_vision.updateCalculations(target, m_desiredX, m_desiredY, m_desiredRot);

            if (m_vision.isAtTarget()) {
                end(false);
            }

            double sidewaysSpeedX = m_vision.getSidewaysSpeedX();
            double forwardSpeedY = m_vision.getForwardSpeedY();
            double rotationSpeed = m_vision.getRotationSpeed();

            m_swerveSubsystem.drive(sidewaysSpeedX, forwardSpeedY, rotationSpeed);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
