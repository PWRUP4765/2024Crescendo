package frc.robot.util.controller;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.error.NoChannelFoundException;
import java.util.List;

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

    public int value;

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

    /**
     * @apiNote DO NOT SET VALUES! THIS IS ONLY FOR CUSTOM CONFIGS!
     * @param newVal the new value of the enum
     */
    public void setValue(int newVal) {
      this.value = newVal;
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

    public int value;

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

    /**
     * @apiNote DO NOT SET VALUES! THIS IS ONLY FOR CUSTOM CONFIGS!
     * @param newVal the new value of the enum
     */
    public void setValue(int newVal) {
      this.value = newVal;
    }
  }

  /**
   * @param port the port of the controller
   * @throws NoChannelFoundException if the channel is invalid that means that some code upstairs is buggy and needs to be fixed
   */
  public LogitechController(int port) /*throws NoChannelFoundException*/ {
    super(port);
    /*if (port < 0) {
      throw new NoChannelFoundException(port);
    }*/
  }

  /**
   * @param values new values
   */
  public void setValues(List<Integer> values) {
    for (int i = 0; i < values.size(); i++) {
      if (i < 12) {
        ButtonEnum.values()[i].setValue(values.get(i));
      } else {
        AxisEnum.values()[i].setValue(values.get(i));
      }
    }
  }
}
