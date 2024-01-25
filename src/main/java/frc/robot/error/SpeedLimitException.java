package frc.robot.error;

public class SpeedLimitException extends Exception {

  final double speed;

  public SpeedLimitException(double speedTriedToSet) {
    super("The speed that you set is too high!");
    this.speed = speedTriedToSet;
  }
}
