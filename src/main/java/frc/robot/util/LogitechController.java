package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.error.NoChannelFoundException;

/**
 * @author goofball06
 * @co-author godbrigero
 */
public class LogitechController extends Joystick {

  /**
   * @implNote the default button enum for the logitec controller
   */
  public enum ButtonEnum {
    X(0),
    A(1),
    B(2),
    Y(3),
    LEFTBUTTON(4),
    RIGHTBUTTON(5),
    LEFTTRIGGER(6),
    RIGHTTRIGGER(7),
    BACKBUTTON(8),
    STARTBUTTON(9),
    LEFTJOYSTICKBUTTON(10),
    RIGHTJOYSTICKBUTTON(11);

    public final int value;

    /**
     * @param val for enum setting
     */
    ButtonEnum(int val) {
      this.value = val;
    }

    /**
     * @apiNote stringifies the enum
     */
    @Override
    public String toString() {
      return (
        "ButtonEnum{" + "name='" + name() + '\'' + ", intValue=" + value + '}'
      );
    }
  }

  /**
   * @apiNote this is for axis channels in a controller for the logitec
   */
  public enum AxisEnum {
    LEFTJOYSTICKX(0),
    LEFTJOYSTICKY(1),
    RIGHTJOYSTICKX(2),
    RIGHTJOYSTICKY(3);

    public final int value;

    /**
     * @param val for setting the port
     */
    AxisEnum(int val) {
      this.value = val;
    }

    /**
     * @apiNote stringifies the enum
     */
    @Override
    public String toString() {
      return (
        "ButtonEnum{" + "name='" + name() + '\'' + ", intValue=" + value + '}'
      );
    }
  }

  /**
   * @param port the port of the controller
   * @throws NoChannelFoundException if the channel is invalid that means that some code upstairs is buggy and needs to be fixed
   */
  public LogitechController(int port) throws NoChannelFoundException {
    super(port);
    if (port < 0) {
      throw new NoChannelFoundException(port);
    }
  }
}
