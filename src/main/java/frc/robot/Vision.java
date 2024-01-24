package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * This implementation of the Vision subsystem expects PhotonVision to be installed on the camera
 * coprocessor system.
 */
public class Vision extends SubsystemBase {
  
  private final PhotonCamera m_photonCamera;

  // PID for turning
  final double ANGULAR_P = 0.02;
  final double ANGULAR_D = 0.0;
  private final PIDController m_turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  // PID for forward motion
  final double LINEAR_P = 1;
  final double LINEAR_D = 0.0;
  private final PIDController m_forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  private Pose2d m_pose2d;

  private double m_lastEstTimestamp = 0;

  private PhotonPoseEstimator m_photonPoseEstimator;

  private AprilTagFieldLayout m_aprilTagFieldLayout;

  public Vision(String cameraName) {

    m_photonCamera = new PhotonCamera(cameraName);

    // load field layout
    m_aprilTagFieldLayout = loadFieldLayout("apriltag-locations.json");

    // Construct PhotonPoseEstimator
    m_photonPoseEstimator= new PhotonPoseEstimator(m_aprilTagFieldLayout, 
                                                    PoseStrategy.CLOSEST_TO_REFERENCE_POSE, 
                                                    m_photonCamera, Constants.VisionConstants.ROBOT_TO_CAMERA);

    m_photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  /**
   * Use the field layout to help idenfity the position of the robot relative to the april tags set in the field.
   */
  private AprilTagFieldLayout loadFieldLayout(String...file) {
    AprilTagFieldLayout tagsFieldLayout = null;
    try {
      if ( file!=null && file.length > 0 ) {
        tagsFieldLayout = AprilTagFieldLayout.loadFromResource(file[0]);
      }
      else {
        // The layout would change per year, so this enum needs to change to pick the right layout file.
        tagsFieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      }
      var alliance = DriverStation.getAlliance();
      tagsFieldLayout.setOrigin(alliance == Alliance.Blue ?
          OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);
    } 
    catch(IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
    }
    return tagsFieldLayout;
  }

  @Override
  public void periodic() {
    // update pose periodically
    //getCurrentPose2d();
  }

  public Optional<Pose3d> getFieldTagPose(int fiducialId) {
      // Get the tag pose from field layout - consider that the layout will be null if it failed to load
      return m_aprilTagFieldLayout == null ? 
                                  Optional.empty() : m_aprilTagFieldLayout.getTagPose(fiducialId);

  }

  /**
   * Gets the current pose or relative position of the robot on the field. It does so based on 
   * identifying the location of an april tag and then relying on its field placement position
   * from the field layout loaded, to determine its own position.
   * 
   * @return pose
   */
  public Pose2d getCurrentPose2d() {

    Pose3d robotPose3d = null;

    // This method will be called once per scheduler run
    VisionTarget bestTarget = getBestTarget();
    if ( bestTarget!= null ) {
      m_lastEstTimestamp = bestTarget.getTimestamp();
      PhotonTrackedTarget trackedTarget = bestTarget.get();
      Transform3d camToTarget = trackedTarget.getBestCameraToTarget();   
      System.out.println("Camera to target: " + camToTarget);   
      var fiducialId = trackedTarget.getFiducialId();
    
      // if an april tag is found, set it for a reference pose. This makes the robot's relative
      // location known given the tag's predefined position on the field.
      Optional<Pose3d> fieldTagPose = getFieldTagPose(fiducialId);
      if (fieldTagPose.isPresent()) {
        // Variables needed to calculate Robot's position
        
        var targetPose = fieldTagPose.get();
        Transform3d camToRobot = Constants.VisionConstants.CAMERA_TO_ROBOT;
        robotPose3d = PhotonUtils.estimateFieldToRobotAprilTag(camToTarget, targetPose, camToRobot);

        // set robot's calculated pose as its initial reference pose when prior reading is null
        if ( m_pose2d == null ) {          
          m_photonPoseEstimator.setReferencePose(robotPose3d);
        }

        Pose2d newPose2d = robotPose3d.toPose2d();
        if ( m_pose2d==null ||  (newPose2d!=null && !m_pose2d.equals(newPose2d)) ) {
          m_pose2d = newPose2d;          
        }
      }
    }
    else {
      Optional<EstimatedRobotPose> currentPose = m_photonPoseEstimator.update();
      if ( currentPose.isPresent() ) {
        m_pose2d = currentPose.get().estimatedPose.toPose2d();
      }      
    }
    return m_pose2d;
  }

  public double getForwardSpeed(VisionTarget target) {
    // Use this range as the measurement we give to the PID controller.
    // -1.0 required to ensure positive PID controller effort _increases_ range
    var range = getRange(target);
    return -m_forwardController.calculate(range, VisionConstants.GOAL_RANGE_METERS);
  }

  public double getRotationSpeed(VisionTarget target) {
    // Also calculate angular power
    // -1.0 required to ensure positive PID controller effort _increases_ yaw
    return m_turnController.calculate(target.get().getYaw(), 0);
  }
    
  public double getRange(VisionTarget visionTarget) {

    return PhotonUtils.calculateDistanceToTargetMeters(
      VisionConstants.CAMERA_HEIGHT_METERS,
      VisionConstants.TARGET_HEIGHT_METERS,
      VisionConstants.CAMERA_PITCH_RADIANS,
      Units.degreesToRadians(visionTarget.get().getPitch()));
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
      Optional<EstimatedRobotPose> visionEst = m_photonPoseEstimator.update();        
      double latestTimestamp = m_photonCamera.getLatestResult().getTimestampSeconds();
      boolean newResult = Math.abs(latestTimestamp - m_lastEstTimestamp) > 1e-5;
      if (newResult) {
        m_lastEstTimestamp = latestTimestamp;
      }
      return visionEst;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return m_photonPoseEstimator.update();
  }

  public VisionTarget getBestTarget() {
    VisionTarget visionTarget = null;
    // Get latest pipeline results
    try {
      var pipelineResult = m_photonCamera.getLatestResult();
      //&& m_lastEstTimestamp != pipelineResult.getTimestampSeconds()
      if (pipelineResult.hasTargets()) {      
        visionTarget = new VisionTarget(pipelineResult.getBestTarget(), pipelineResult.getTimestampSeconds());
      }
    }
    catch(Exception ex) {
      System.out.println("No vision data: " + ex.getMessage());
    }

    return visionTarget;
  }

  // Avoid exposing PhotonVision classes outside of this subsystem
  public static class VisionTarget {
    private PhotonTrackedTarget m_trackedTarget;
    private double m_timestamp;

    private VisionTarget(PhotonTrackedTarget target, double timestamp) {
      m_trackedTarget = target;
      m_timestamp = timestamp;
    }

    PhotonTrackedTarget get() {
      return m_trackedTarget;
    }    

    double getTimestamp() {
      return m_timestamp;
    }
  }
}