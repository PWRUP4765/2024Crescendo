import frc.robot.inter.ReplayLoggerDevice;
import frc.robot.subsystems.ReplayLoggerSubsystem;
import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import util.TestUtils;

/**
 * @author godbrigero
 */
public class LidarModulesTest {

  public String fileName = "TestFile";

  @Test
  void lidarLoggerTest() {
    double[][] fakeScans = TestUtils.generateFakeScans(50);
    List<Double> scan = new ArrayList<>();

    ReplayLoggerDevice fakeDevice = new ReplayLoggerDevice() {
      @Override
      public List<Double> getData(boolean d) {
        return scan;
      }
    };

    ReplayLoggerDevice[] devices = new ReplayLoggerDevice[] { fakeDevice };

    ReplayLoggerSubsystem logger = null;
    try {
      logger = new ReplayLoggerSubsystem(devices, fileName, 0);
    } catch (IOException e) {
      e.printStackTrace();
    }

    for (double[] curScan : fakeScans) {
      for (double i : curScan) {
        scan.add(i);
      }

      logger.forceLog();
      scan.clear();
    }
  }

  @Test
  void testWrittenData() throws Exception {
    double[][] fakeScans = TestUtils.generateFakeScans(50);
    List<Double> scan = new ArrayList<>();

    ReplayLoggerDevice fakeDevice = new ReplayLoggerDevice() {
      @Override
      public List<Double> getData(boolean d) {
        return scan;
      }
    };

    ReplayLoggerDevice[] devices = new ReplayLoggerDevice[] { fakeDevice };

    ReplayLoggerSubsystem logger = null;
    try {
      logger = new ReplayLoggerSubsystem(devices, fileName, 0);
    } catch (IOException e) {
      e.printStackTrace();
    }

    for (double[] curScan : fakeScans) {
      for (double i : curScan) {
        scan.add(i);
      }

      logger.forceLog();
      scan.clear();
    }

    try (
      FileInputStream fileInputStream = new FileInputStream(new File(fileName));
      DataInputStream dataInputStream = new DataInputStream(fileInputStream)
    ) {
      int scanCount = 0;
      while (dataInputStream.available() > 0) {
        int amtScans = dataInputStream.readInt();
        for (int i = 0; i < amtScans; i++) {
          double value = dataInputStream.readDouble();
          if (fakeScans[scanCount][i] != value) {
            throw new Exception("Incorrect read!");
          }
        }

        scanCount++;
      }
    } catch (IOException e) {
      e.printStackTrace();
    }
  }
}
