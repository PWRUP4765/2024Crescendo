package frc.robot.inter;

import frc.robot.error.LimitException;

/**
 * @author godbrigero
 */
public interface BalanceMotorAtSpot {
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
