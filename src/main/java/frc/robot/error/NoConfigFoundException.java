package frc.robot.error;

/**
 * @author godbrigero
 */
public class NoConfigFoundException extends Exception {

  public NoConfigFoundException() {
    super("No config found with this name!");
  }
}
