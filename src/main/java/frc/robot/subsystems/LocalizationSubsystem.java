package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
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

  // https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/Apriltag_Images_and_User_Guide.pdf
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  public LocalizationSubsystem(VisionSubsystem vision, SwerveModule[] modules) {
    m_gyro = new AHRS(I2C.Port.kMXP);
    m_vision = vision;

    m_frontLeftModule = modules[0];
    m_frontRightModule = modules[1];
    m_rearLeftModule = modules[2];
    m_rearRightModule = modules[3];

    // Locations for the swerve drive modules relative to the robot center.
    // X is equal to the Y, and the Y is inverted
    //0.2413 meters on each axis from the center of the robot these should be correct
    Translation2d m_frontLeftLocation = new Translation2d(0.2413, 0.2413); 
    Translation2d m_frontRightLocation = new Translation2d(0.2413, -0.2413);
    Translation2d m_rearLeftLocation = new Translation2d(-0.2413, 0.2413);
    Translation2d m_rearRightLocation = new Translation2d(-0.2413, -0.2413);

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
        getModulePositions(),
        new Pose2d(0, 0, new Rotation2d()) //THIS VALUE ALSO NEEDS TO BE CORRECTED
      );
  }

  /**
   * @apiNote this is needed for simpler code
   * @return positions of all wheels
   */
  private SwerveModulePosition[] getModulePositions() {
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
  private Pose2d getOdometryPosition() {
    return m_odometry.getPoseMeters();
  }

  /**
   * @apiNote this checks if the limelight detects a target, if it does it gets the position of the robot relative to the april tag
   * @return the robot position
   */
  private Pose2d getVistionPosition() {
    VisionTarget target = m_vision.findBestTarget();

    int id = target.getID();
    Pose3d targetPos = aprilTagFieldLayout.getTagPose(id).isPresent() ? aprilTagFieldLayout.getTagPose(id).get() : null;

    if (targetPos == null) {
      return null;
    }

    double targetHeight = targetPos.getZ();
    double targetPitch = targetPos.getRotation().getAngle();
    Pose2d targetPose = targetPos.toPose2d();

    Transform2d cameraToRobot = new Transform2d(0.2892, 0, new Rotation2d()); 
    // the position of the camera from the ground and the center of the robot
    // 0.2892 meters behind the robot this also should be correct

    Pose2d robotPos = PhotonUtils.estimateFieldToRobot(
         VisionConstants.kCAMERA_HEIGHT_METERS, 
         targetHeight, 
         VisionConstants.kCAMERA_PITCH, 
         targetPitch, 
         Rotation2d.fromDegrees(-target.getYaw()), 
         m_gyro.getRotation2d(), 
         targetPose, 
         cameraToRobot); 

    return robotPos;
  }

  /**
   * @apiNote This sets the odometry to whatever position we want it to 
   */
  private void setOdomPosition(Pose2d pos){
    this.m_odometry.resetPosition(this.m_gyro.getRotation2d(), getModulePositions(), pos);
  }

  /**
   * @apiNote this is needed bc according to the docs, "...you need to update the odometry position periodically..."
   */
  private void updateOdomPosition() {
    this.m_odometry.update(this.m_gyro.getRotation2d(), getModulePositions());
  }

  /**
   * @return returns the current position of the robot in a Pose2d object
   */
  public Pose2d getPosition() {
    return position;
  }

  /**
   * @apiNote This uses the Odometry as the base for our localization, but if we detect an april tag we will use that data to reset our localization
   */
  public void updatePosition() {
    updateOdomPosition();

    position = this.getVistionPosition();

    if (position == null) {
      position = this.getOdometryPosition();
    }else{
      this.setOdomPosition(position);
    }
  }
}
