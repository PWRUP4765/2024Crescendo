package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.error.LimitException;
import frc.robot.error.NoChannelFoundException;

/**
 * @author godbrigero
 */
public class ClimbArmSubsystem extends SubsystemBase {

  CANSparkMax sparkMax;
  AbsoluteEncoder encoder;
  SparkPIDController pid;
  final TalonSRX talon;

  /**
   * @param channel     the motor channel this is FOR DEBUG PURPOSES
   * @throws NoChannelFoundException if the channel is below the min threshhold 0
   *                                 for now it will throw this
   */
  public ClimbArmSubsystem(int channel) {
    talon = new TalonSRX(channel);
  }

  /**
   * @throws LimitException will throw an exception if the speed if above / below
   *                        the min threshhold
   */
  public void setSpeed(double speed) throws LimitException {
    this.talon.set(TalonSRXControlMode.PercentOutput, speed);
  }

  /**
   * @return will return the top limit switch state
   */
  public boolean isArmOnTop() {
    return talon.isFwdLimitSwitchClosed() == 1;
  }

  public boolean isArmOnBottom() {
    return talon.isRevLimitSwitchClosed() == 1;
  }

  /**
   * @param speedDouble the speed that the user is trying to set
   * @return returns if the speed is not exceeding the limit true / false
   */
  private boolean checkSpeed(double speedDouble) {
    return speedDouble < -1.0 || speedDouble > 1.0;
  }

  /**
   * @apinote stops the motor.
   */
  public void stopMotor() {
    this.talon.set(TalonSRXControlMode.PercentOutput, 0);
  }

  /**
   * @return this will return the speed of the motor in %
   */
  public double getCurrentSetSpeedPerc() {
    double nonPerc = this.sparkMax.get();
    return nonPerc * 100;
  }

  /**
   * @return this will return values from -1 to 1 idk y u need dis bc this is %
   *         based.
   */
  public double getCurrentSpeedDouble() {
    return this.sparkMax.get();
  }

  /**
   * @return This CAN be negative. This can also be positive. It will return the
   *         amt of revolutions that the external
   *         device counts
   */
  public double getRevSinceStart() {
    return encoder.getPosition();
  }

  /**
   * @apiNote this resets the encoder's position to 0 res
   */
  public void resetPosition() {
    encoder.setInverted(true);
    encoder.setZeroOffset(0.606);
  }

  public SparkPIDController getPid() {
    return this.sparkMax.getPIDController();
  }

  /**
   * @apiNote (+) = up, (-) = down, 15 speed is minimum it can go up or it will stay in same spot
   */
  public void tick(boolean isOffline, boolean buttonState) {
    try {
      if (!isOffline) {
        if (buttonState) {
          setSpeed(-20);
        } else {
          setSpeed(0);
        }

        return;
      }

      stopMotor();
    } catch (LimitException e) {
      throw new RuntimeException(e);
    }
  }
}
