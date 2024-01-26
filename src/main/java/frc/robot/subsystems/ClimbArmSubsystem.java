package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbArmConstants;
import frc.robot.error.LimitException;
import frc.robot.error.NoChannelFoundException;

/**
 * @author godbrigero
 */
public class ClimbArmSubsystem extends SubsystemBase {

  final CANSparkMax sparkMax;
  final RelativeEncoder encoder;

  /**
   * @param channel     the motor channel this is FOR DEBUG PURPOSES
   * @param isBrushless not sure what this is but it will set the
   *                    MotorType.kBrushed if "isBrushless == false" and
   *                    MotorType.kBrushless if "isBrushless == true"
   * @throws NoChannelFoundException if the channel is below the min threshhold 0
   *                                 for now it will throw this
   */
  public ClimbArmSubsystem(int channel, boolean isBrushless)
    throws NoChannelFoundException {
    // @this might have to be re-worked since the channels may be > also.
    if (channel < 0) throw new NoChannelFoundException(channel);

    sparkMax =
      new CANSparkMax(
        ClimbArmConstants.kClimbArmMotorPort,
        ClimbArmConstants.kClimbArmMotorIsBrushless
          ? MotorType.kBrushless
          : MotorType.kBrushed
      );

    this.encoder = sparkMax.getEncoder();
  }

  /**
   * @param speedPerc the % of the max motor speed that you want to set
   * @throws LimitException will throw an exception if the speed if above / below
   *                        the min threshhold
   */
  public void setSpeed(double speedPerc) throws LimitException {
    double speed = speedPerc / 100;
    if (checkSpeed(speed)) throw new LimitException(
      speedPerc,
      this.getClass().getName()
    );

    this.sparkMax.set(speed);
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
    this.sparkMax.stopMotor();
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
    encoder.setPosition(0);
  }
}
