package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionTarget;
import frc.robot.util.controller.FlightModule;
import frc.robot.util.controller.FlightStick;

public class TeleGoToAprilTag extends Command {

  VisionSubsystem m_vision;
  SwerveSubsystem m_swerveSubsystem;

  double m_desiredX, m_desiredY, m_desiredRot;
  FlightModule m_controller;

  VisionTarget m_lastTarget;
  int timesSinceLastSight = 100;

  public TeleGoToAprilTag(
    FlightModule controller,
    SwerveSubsystem swerveDrive,
    VisionSubsystem vision,
    double x,
    double y,
    double rot
  ) {
    m_vision = vision;
    m_swerveSubsystem = swerveDrive;
    m_controller = controller;

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
    var target = m_vision.findBestTarget();

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

    } else {
      double x = m_controller.rightFlightStick.getRawAxis(FlightStick.AxisEnum.JOYSTICKX.value);
      double y = m_controller.rightFlightStick.getRawAxis(FlightStick.AxisEnum.JOYSTICKY.value) * -1;
      double z = m_controller.leftFlightStick.getRawAxis(FlightStick.AxisEnum.JOYSTICKROTATION.value);
      m_swerveSubsystem.joystickDrive(x, y, z);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
