package frc.robot.inter;

import frc.robot.error.LimitException;

/**
 * @author godbrigero
 * @apiNote can be useful later to make libraries in which multiple motors can work
 */
public interface MotorInterface {
  public double getMotorCurPosition();

  /**
   * @param speedPerc in %
   */
  public void setMotorSpeed(double speedPerc) throws LimitException;

  /**
   * @return the motor speed in %
   */
  public double getCurMotorSpeedPerc();
}
