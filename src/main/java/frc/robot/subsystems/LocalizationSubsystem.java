package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionTarget;
import org.photonvision.PhotonUtils;

public class LocalizationSubsystem extends SubsystemBase {

  private Pose2d position;
  private AHRS m_gyro;
  VisionSubsystem m_vision;
  SwerveDriveOdometry m_odometry;

  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_rearLeftModule;
  private final SwerveModule m_rearRightModule;

  public LocalizationSubsystem(VisionSubsystem vision, SwerveModule[] modules) {
    m_gyro = new AHRS(I2C.Port.kMXP);
    m_vision = vision;

    m_frontLeftModule = modules[0];
    m_frontRightModule = modules[1];
    m_rearLeftModule = modules[2];
    m_rearRightModule = modules[3];

    // Locations for the swerve drive modules relative to the robot center.
    // THESE VALUES ARE NOT CORRECT AND NEED TO BE FOUND IN METERS
    Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    Translation2d m_rearLeftLocation = new Translation2d(-0.381, 0.381);
    Translation2d m_rearRightLocation = new Translation2d(-0.381, -0.381);

    // Creating my kinematics object using the module locations
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation,
      m_frontRightLocation,
      m_rearLeftLocation,
      m_rearRightLocation
    );

    // Creating my odometry object from the kinematics object and the initial wheel positions.
    // Here, our starting pose is 0 meters along the long end of the field and at the bottom
    // of the field along the short end, facing the opposing alliance wall.
    this.m_odometry =
      new SwerveDriveOdometry(
        m_kinematics,
        m_gyro.getRotation2d(),
        gModulePositions(),
        new Pose2d(0, 0, new Rotation2d())
      );
  }

  /**
   * @apiNote this is needed for simpler code
   * @return positions of all wheels
   */
  public SwerveModulePosition[] gModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeftModule.getPosition(),
      m_frontRightModule.getPosition(),
      m_rearLeftModule.getPosition(),
      m_rearRightModule.getPosition(),
    };
  }

  /**
   * @return a Position 2D object that tells the x, y and, angle of the robot in METERS and RADIANS using the Odometry Class
   */
  public Pose2d getOdometryPosition() {
    return m_odometry.getPoseMeters();
  }

  /**
   * @return a Position 2D object that tells the x, y and, angle of the robot in METERS and RADIANS using the NavX
   */
  public Pose2d getGyroPosition() {
    return new Pose2d(
      (double) m_gyro.getDisplacementX(),
      (double) m_gyro.getDisplacementY(),
      new Rotation2d(m_gyro.getAngle() * ((2 * Math.PI) / 360))
    );
  }

  /**
   * WIP - this function is suppposed to find the position of the robot when the liime light detects and april tag
   */
  public Pose2d getVistionPosition(String color) {
    // Pose2d robotPose = PhotonUtils.estimateFieldToRobot(
    //     kCameraHeight, kTargetHeight, kCameraPitch, kTargetPitch, Rotation2d.fromDegrees(-target.getYaw()), gyro.getRotation2d(), targetPose, cameraToRobot);

    VisionTarget target = m_vision.findBestTarget();

    double xPos = color == "red"
      ? FieldConstants.kRedAmpXPosition + target.getX() // This math is probably not correct, this is also assuming target.getX or Y returns a double in METERS
      : FieldConstants.kBlueAmpXPosition + target.getX();

    double yPos = color == "red"
      ? FieldConstants.kRedAmpYPosition + target.getX()
      : FieldConstants.kBlueAmpYPosition + target.getX();

    Rotation2d angle = new Rotation2d(target.getYaw()); // Assumes target.getYaw returns a angle in RADIANS

    return new Pose2d(xPos, yPos, angle);
  }

  /**
   * @apiNote this is needed bc according to the docs, "...you need to update the odometry position periodically..."
   */
  private void updateOdomPosition() {
    this.m_odometry.update(this.m_gyro.getRotation2d(), gModulePositions());
  }

  /**
   * @return returns the current position of the robot in a Pose2d object
   */
  public Pose2d getPosition() {
    return position;
  }

  // WIP - this should update the position of the robot by averaging all of the positions of the NavX, Odometry, and Vision into one
  public void updatePosition() {
    updateOdomPosition();

    Pose2d odom = this.getOdometryPosition();
    // Not sure what to put for color since I don't really understand what that is used for
    Pose2d gyro = this.getGyroPosition();

    Pose2d vision = this.getVistionPosition("red");

    double x = 0;
    double y = 0;
    double rotRad = 0;
    if (vision == null) {
      x = (odom.getX() + gyro.getX()) / 2;
      y = (odom.getY() + gyro.getY()) / 2;
      rotRad =
        (odom.getRotation().getRadians() + gyro.getRotation().getRadians()) / 2;
    } else {
      x = (odom.getX() + vision.getX() + gyro.getX()) / 3;
      y = (odom.getY() + vision.getY() + gyro.getY()) / 3;
      rotRad =
        (
          odom.getRotation().getRadians() +
          vision.getRotation().getRadians() +
          gyro.getRotation().getRadians()
        ) /
        3;
    }

    this.position = new Pose2d(x, y, new Rotation2d(rotRad));
  }
}
