package frc.robot.error;

/**
 * @author godbrigero
 */
public class LimitException extends Exception {

  final double speed;
  final String subsystemBroken;

  public LimitException(double speedTriedToSet, String subsystemName) {
    super(
      "The limit that you set has been broken! This is in " + subsystemName
    );
    this.speed = speedTriedToSet;
    this.subsystemBroken = subsystemName;
  }
}
