package frc.robot.hardware;

import edu.wpi.first.wpilibj.I2C;

public class HardwareComponents {
    
    public static NavX gyro = new NavX(I2C.Port.kMXP);
    // public static IntakeSensor intakeSensor = new IntakeSensor();
}
