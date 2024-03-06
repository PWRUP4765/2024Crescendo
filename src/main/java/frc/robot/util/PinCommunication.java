package frc.robot.util;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;

public class PinCommunication {

  private static SPI spiPin = new SPI(SPI.Port.kOnboardCS1);

  public static void sendOnline() {
    byte[] dataToSend = { 0x01 };
    spiPin.write(dataToSend, dataToSend.length);
  }
}
