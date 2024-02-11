package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveConstants;

public class SwerveDummy extends SwerveSubsystem {

  public SwerveDummy() {}

  public void joystickDrive(double x, double y, double r) {}

  public void drive(double x, double y, double r) {}

  public void reset() {}
}
