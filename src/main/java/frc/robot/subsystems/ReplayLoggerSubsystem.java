package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.inter.ReplayLoggerDevice;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.List;

/**
 * @author godbrigero
 * @purpose to log lidar data MAINLY but can be used for other devices
 */
public class ReplayLoggerSubsystem extends SubsystemBase {

  final double[][] deviceData;
  final String fileName;
  final ReplayLoggerDevice[] devices;
  final int ticksPerUpdate;
  final DataOutputStream writer;

  int currentTicks = 0;
  boolean state;

  /**
   * @param devices the interface of devices that you want to use
   * @param fileName the name of the file to write to
   * @param ticksPerUpdate the ticks between writes to file (1 tick = 25ms)
   * @throws IOException if no file is found with the name
   */
  public ReplayLoggerSubsystem(
    ReplayLoggerDevice[] devices,
    String fileName,
    int ticksPerUpdate
  ) throws IOException {
    this.deviceData = new double[devices.length][];
    this.fileName = fileName;
    this.devices = devices;

    this.setDefaultCommand(new RunCommand(() -> tick25Ms(), this));
    this.ticksPerUpdate = ticksPerUpdate;

    File file = new File(fileName);
    if (!file.exists()) {
      file.createNewFile();
    } else {
      file.delete();
      file.createNewFile();
    }

    this.writer = new DataOutputStream(new FileOutputStream(file));
  }

  /**
   * @param state so u can start / stop logger (boolean)
   */
  public void startStopLogger(boolean state) {
    this.state = state;
  }

  /**
   * @description tick each 25ms also does all the writing
   */
  public void tick25Ms() {
    if (!state) return;

    if (currentTicks >= ticksPerUpdate) {
      currentTicks = 0;
    } else {
      currentTicks++;
      return;
    }

    log();
  }

  /**
   * @apiNote force logs even if there is no info
   */
  public void forceLog() {
    log();
  }

  /**
   * @apiNote does the logging there so that forceLog can exist
   */
  void log() {
    for (int i = 0; i < devices.length; i++) {
      double[] dataInit = devices[i].getData();
      if (dataInit != null) {
        deviceData[i] = new double[dataInit.length];
        for (int j = 0; j < dataInit.length; j++) {
          deviceData[i][j] = dataInit[j];
        }

        continue;
      }

      List<Double> dataInitList = devices[i].getData(false);
      if (dataInitList != null) {
        deviceData[i] = new double[dataInitList.size()];
        for (int j = 0; j < dataInitList.size(); j++) {
          deviceData[i][j] = dataInitList.get(j);
        }
      }
    }

    try {
      for (double[] scans : deviceData) {
        writer.writeInt(scans.length);
        for (int j = 0; j < scans.length; j++) {
          writer.writeDouble(scans[j]);
        }
      }
    } catch (IOException e) {
      e.printStackTrace();
    }
  }
}
