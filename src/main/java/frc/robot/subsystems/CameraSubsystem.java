// package frc.robot.subsystems;

// import org.photonvision.PhotonCamera;
// import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.networktables.GenericEntry;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

// public class CameraSubsystem {
//     //https://docs.photonvision.org/en/latest/docs/programming/photonlib/getting-target-data.html
//     private String cameraName;
//     private PhotonCamera camera;

//     private double
//     targetX,
//     targetY,
//     targetZ,
//     targetRoll,
//     targetPitch,
//     targetYaw;


//     private ShuffleboardTab sb_tab;

//     private GenericEntry
//     sb_targetX,
//     sb_targetY,
//     sb_targetZ,
//     sb_targetRoll,
//     sb_targetPitch,
//     sb_targetYaw;


//     public CameraSubsystem(String cameraName) {
//         camera = new PhotonCamera(cameraName);

//         createShuffleboardTab();
//     }

//     public void updateTarget() {
//         var result = camera.getLatestResult();
//         if (!result.hasTargets()) { return;}
//         PhotonTrackedTarget target = result.getBestTarget();

//         Transform3d bestCameraToTarget = target.getBestCameraToTarget();

//         targetX = bestCameraToTarget.getX();
//         targetY = bestCameraToTarget.getY();
//         targetZ = bestCameraToTarget.getZ();
//         targetRoll = bestCameraToTarget.getRotation().getX();
//         targetPitch = bestCameraToTarget.getRotation().getY();
//         targetYaw = bestCameraToTarget.getRotation().getZ();

//         updateShuffleboardTab(targetX, targetY, targetZ, targetRoll, targetPitch, targetYaw);
//     }

//     public void createShuffleboardTab() {
//         sb_tab = Shuffleboard.getTab(cameraName);
        
//         sb_targetX = sb_tab.add("targetX", 0).getEntry();
//         sb_targetY = sb_tab.add("targetY", 0).getEntry();
//         sb_targetZ = sb_tab.add("targetZ", 0).getEntry();
//         sb_targetRoll = sb_tab.add("targetRoll", 0).getEntry();
//         sb_targetPitch = sb_tab.add("targetPitch", 0).getEntry();
//         sb_targetYaw = sb_tab.add("targetYaw", 0).getEntry();
//     }

//     public void updateShuffleboardTab(
//       double x,
//       double y,
//       double z,
//       double roll,
//       double pitch,
//     double yaw) {

//         sb_targetX.setDouble(x);
//         sb_targetY.setDouble(y);
//         sb_targetZ.setDouble(z);
//         sb_targetRoll.setDouble(roll);
//         sb_targetPitch.setDouble(pitch);
//         sb_targetYaw.setDouble(yaw);
//     }
// }
