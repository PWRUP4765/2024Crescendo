package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.error.NoChannelFoundException;

public class LogitechController extends Joystick {
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

        ButtonEnum(int val) {
            this.value = val;
        }


        @Override
        public String toString() {
            return "ButtonEnum{" +
                    "name='" + name() + '\'' +
                    ", intValue=" + value +
                    '}';
        }
    }

    public enum AxisEnum {

        LEFTJOYSTICKX(0),
        LEFTJOYSTICKY(1),
        RIGHTJOYSTICKX(2),
        RIGHTJOYSTICKY(3);

        public final int value;
        
        AxisEnum(int val) {
            this.value = val;
        }

        @Override
        public String toString() {
            return "ButtonEnum{" +
                    "name='" + name() + '\'' +
                    ", intValue=" + value +
                    '}';
        }
    }

    public LogitechController(int port) throws NoChannelFoundException {
        super(port);
        
        if (port < 0) {
            throw new NoChannelFoundException(port);
        }
    }
}
