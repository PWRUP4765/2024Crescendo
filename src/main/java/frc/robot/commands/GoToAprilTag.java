package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class GoToAprilTag extends Command {

  VisionSubsystem m_vision;
  SwerveSubsystem m_swerveDrive;

  public GoToAprilTag(SwerveSubsystem swerveDrive, VisionSubsystem vision) {
    m_vision = vision;
    m_swerveDrive = swerveDrive;
  }

  @Override
  public void execute() {
    var target = m_vision.findBestTarget();
    if (target != null) {
      var forwardSpeedX = m_vision.calculateForwardSpeedX(target);
      var forwardSpeedSpeedY = m_vision.calculateForwardSpeedY(target);
      var rotationSpeed = m_vision.calculateRotationSpeed(target);
      m_swerveDrive.drive(forwardSpeedX, forwardSpeedSpeedY, rotationSpeed);
    }
  }
}
