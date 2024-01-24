package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;

/**
 * Track and estimate a robot's pose on the field using vision and drivetrain movement.
 */
public class PoseOdometryEstimation extends SubsystemBase{

  private final DifferentialDrivePoseEstimator m_poseEstimator;

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  // Also show a field diagram
  private final Field2d m_field2d = new Field2d();

  // Dependent subsystems
  private final Gyro m_gyro;
  private final Vision m_vision;
  private final Drivetrain m_drivetrain;

  public PoseOdometryEstimation(Drivetrain drivetrain, Gyro gyro, Vision vision) {

    // Set instance variables for later use of these modules
    m_gyro = gyro;
    m_vision = vision;
    m_drivetrain = drivetrain;

    m_poseEstimator = new DifferentialDrivePoseEstimator(
                  DriveConstants.kDriveKinematics, 
                  m_gyro.getRotation2d(), 
                  m_drivetrain.getLeftDistanceMeter(), 
                  m_drivetrain.getRightDistanceMeter(), 
                  new Pose2d(), 
                  VisionConstants.stateStdDevs,
                  VisionConstants.visionMeasurementStdDevs);

    m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), 
              m_drivetrain.getLeftDistanceMeter(),m_drivetrain.getRightDistanceMeter());
    
    // Add field as a viewable on the dashboard for correlating position via the drivetrain.
    SmartDashboard.putData("field", m_field2d);  
  }

  /**
   * Update pose every 20ms and accordingly update the robot's position on the field.
   */
  @Override
  public void periodic() {
    
    Optional<EstimatedRobotPose> visionEstimatedPose = m_vision.getEstimatedGlobalPose();
    if ( visionEstimatedPose.isPresent() ) {
      EstimatedRobotPose visionBasedPose = visionEstimatedPose.get();
      m_poseEstimator.addVisionMeasurement(visionBasedPose.estimatedPose.toPose2d(), 
                                           visionBasedPose.timestampSeconds);
    }

    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(), m_drivetrain.getLeftDistanceMeter(),m_drivetrain.getRightDistanceMeter());
    m_poseEstimator.update(m_gyro.getRotation2d(), m_drivetrain.getLeftDistanceMeter(),m_drivetrain.getRightDistanceMeter());

    // update position on field 
    m_field2d.setRobotPose(m_poseEstimator.getEstimatedPosition());
  }

    /**
   * Resets the odometry to the specified pose
   * @param pose The pose to which to set the odometry
   */
  public void resetOdometry(Pose2d pose) {
    m_drivetrain.resetEncoders();
    m_odometry.resetPosition(m_gyro.getRotation2d(), 
          m_drivetrain.getLeftDistanceMeter(), m_drivetrain.getRightDistanceMeter(), pose);
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }
}