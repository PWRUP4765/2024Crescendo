package frc.robot.util.controller;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ControllerBase extends Joystick {
    
    public ButtonList Buttons;

    public ControllerBase(int port) {
        super(port);
    }
    public class ButtonList {

        public class Button {

            public int value;

            public JoystickButton getJoystickButton() {
                return new JoystickButton(ControllerBase.this, this.value);
            }

            public boolean isButtonPressed() {
                return ControllerBase.this.getRawButton(value);
            }

            public Button(int val) {
                this.value = val;
            }
        }
    }

    public class AxisList {
        public class Axis {
        
            public int value;
        
            public double getAxisValue() {
                return ControllerBase.this.getRawAxis(value);
            }
        
            public Axis(int val) {
                this.value = val;
            }
        }
    }
}
