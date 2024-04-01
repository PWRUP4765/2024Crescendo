package frc.robot.hardware;

import edu.wpi.first.wpilibj.I2C;

public class HardwareComponents {
    
    public static navX gyro = new navX(I2C.Port.kMXP);
}
