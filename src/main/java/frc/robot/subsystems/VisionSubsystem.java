package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {

  PhotonCamera m_camera;
  PhotonCamera m_result;
  PhotonPipelineResult m_pipline = new PhotonPipelineResult();

  private final PIDController m_fowardController = new PIDController(
    VisionConstants.ANGULAR_P,
    0,
    VisionConstants.ANGULAR_D
  );

  private final PIDController m_rotationController = new PIDController(
    VisionConstants.LINEAR_P,
    0,
    VisionConstants.ANGULAR_D
  );

  public VisionSubsystem(String cameraname) {
    m_camera = new PhotonCamera(cameraname);
  }

  public VisionTarget findBestTarget() {
    VisionTarget target = null;
    var pipelineResult = m_camera.getLatestResult();
    if (pipelineResult != null) {
      target = new VisionTarget(pipelineResult.getBestTarget());
    }
    return target;
  }

  public Double calculateForwardSpeedX(VisionTarget target) {
    var axis = target.getX();
    // -1.0 required to ensure positive PID controller effort _increases_ range
    return -m_fowardController.calculate(
      axis,
      VisionConstants.GOAL_RANGE_METERS
    );
  }

  public Double calculateForwardSpeedY(VisionTarget target) {
    var axis = target.getY();
    // -1.0 required to ensure positive PID controller effort _increases_ range
    return -m_fowardController.calculate(
      axis,
      VisionConstants.GOAL_RANGE_METERS
    );
  }

  public Double calculateRotationSpeed(VisionTarget target) {
    // -1.0 required to ensure positive PID controller effort _increases_ yaw
    return -m_rotationController.calculate(
      getRange(target),
      VisionConstants.GOAL_RANGE_METERS
    );
  }

  public double getRange(VisionTarget target) {
    // First calculate range
    double range = PhotonUtils.calculateDistanceToTargetMeters(
      VisionConstants.CAMERA_HEIGHT_METERS,
      VisionConstants.TARGET_HEIGHT_METERS,
      VisionConstants.TARGET_HEIGHT_METERS,
      Units.degreesToRadians(target.getPitch())
    );
    return range;
  }

  // Avoid exposing PhotonVision classes outside of this subsystem
  public class VisionTarget {

    private PhotonTrackedTarget m_target;
    Transform3d m_transform3d;
    double m_pitch, m_yaw;

    VisionTarget(PhotonTrackedTarget target) {
      m_target = target;
      m_transform3d = m_target.getBestCameraToTarget();
      m_pitch = m_target.getPitch();
      m_yaw = m_target.getYaw();
    }

    public double getX() {
      return m_transform3d.getX();
    }

    public double getY() {
      return m_transform3d.getY();
    }

    public double getPitch() {
      return m_pitch;
    }

    public double getYaw() {
      return m_yaw;
    }
  }
}
