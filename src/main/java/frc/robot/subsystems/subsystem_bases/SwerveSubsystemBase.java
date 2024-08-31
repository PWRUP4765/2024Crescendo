package frc.robot.subsystems.subsystem_bases;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystemBase extends SubsystemBase {
    
    public SwerveSubsystemBase() {}

    public void joystickDrive(double x, double y, double r) {}

    public void drive(double x, double y, double r) {}

    public void drive(double x, double y, double r, double tempSpeedMultiplier) {}

    public void setDesiredDirection(double direction) {}

    public void setSpeedMultiplier(double speedMultiplier) {}

    public void reset() {}

    public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
      new SwerveModulePosition(0, new Rotation2d(0)),
      new SwerveModulePosition(0, new Rotation2d(0)),
      new SwerveModulePosition(0, new Rotation2d(0)),
      new SwerveModulePosition(0, new Rotation2d(0)),
    };
  }
}
