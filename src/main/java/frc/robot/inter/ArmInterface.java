package frc.robot.inter;

import frc.robot.Constants;

public interface ArmInterface {
  double getCurArmPosition();
  void lockArm();
  void unlockArm();
}
