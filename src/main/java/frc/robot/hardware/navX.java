package frc.robot.hardware;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.util.MathFunc;

public class navX extends AHRS {

    public navX(SPI.Port spi_port_id) {
        super(spi_port_id);
    }

    public navX(I2C.Port i2c_port_id) {
        super(i2c_port_id);
    }

    public navX(SerialPort.Port serial_port_id) {
        super(serial_port_id);
    }

    @Override
    public double getAngle() {
        return MathFunc.plusMinus180(super.getAngle());
    }

    public double getUnadjustedAngle() {
        return MathFunc.plusMinus180(getAngle() - super.getAngleAdjustment());
    }

    public void setCurrentAngle(double adjustment) {
        super.setAngleAdjustment(-getUnadjustedAngle() + adjustment);
    }
}
