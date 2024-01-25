package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.error.NoChannelFoundException;
import frc.robot.error.SpeedLimitException;

public class ClimbArmSubsystem extends SubsystemBase {

  final CANSparkMax sparkMax;
  final RelativeEncoder encoder;

  public ClimbArmSubsystem(int channel, boolean isBrushless)
    throws NoChannelFoundException {
    // @this might have to be re-worked since the channels may be > also.
    if (channel < 0) throw new NoChannelFoundException(channel);
    sparkMax =
      new CANSparkMax(
        channel,
        isBrushless ? MotorType.kBrushless : MotorType.kBrushed
      );
    this.encoder = sparkMax.getEncoder();
  }

  // -100% to 100%
  public void setSpeed(double speedPerc) throws SpeedLimitException {
    double speed = speedPerc / 100;
    if (checkSpeed(speed)) throw new SpeedLimitException(speedPerc);

    this.sparkMax.set(speed);
  }

  private boolean checkSpeed(double speedDouble) {
    return speedDouble < -1.0 || speedDouble > 1.0;
  }

  public void stopMotor() {
    this.sparkMax.stopMotor();
  }

  public double getCurrentSetSpeedPerc() {
    double nonPerc = this.sparkMax.get();
    return nonPerc * 100;
  }

  public double getCurrentSpeedDouble() {
    return this.sparkMax.get();
  }

  public double getRevSinceStart() {
    return encoder.getCountsPerRevolution();
  }
}
