package util;

import java.util.Arrays;
import java.util.Random;

public class TestUtils {

  public static Random randomInstance = new Random();

  public static double[][] generateFakeScans(int amount) {
    double[][] scans = new double[amount][];
    for (int i = 0; i < amount; i++) {
      scans[i] = getLidarScanFake(360);
    }

    return scans;
  }

  public static double[] getLidarScanFake(int amtScansPerRev) {
    double[] scans = new double[amtScansPerRev];
    scans[0] = getScanDistance(15);
    for (int i = 1; i < scans.length; i++) {
      scans[i] = getScanDistance(scans[i - 1]);
    }

    return scans;
  }

  public static double[] getLidarScanFake(
    int amtScansPerRev,
    int randomWindow
  ) {
    int amtScans = randomNumberBetweenInt(
      amtScansPerRev - randomWindow,
      amtScansPerRev + randomWindow
    );

    double[] scans = new double[amtScans];
    scans[0] = getScanDistance(15);
    for (int i = 1; i < scans.length; i++) {
      scans[i] = getScanDistance(scans[i - 1]);
    }

    return scans;
  }

  public static double getScanDistance(double prevScanDist) {
    return getScanDistance(prevScanDist, 1);
  }

  public static double getScanDistance(double prevScanDist, double addCoef) {
    return randomNumberBetween(prevScanDist - addCoef, prevScanDist + addCoef);
  }

  public static double randomNumberBetween(double max, double min) {
    double randomNumber = (Math.random() * (max - min)) + min;
    return randomNumber;
  }

  public static int randomNumberBetweenInt(int max, int min) {
    int randomNumber = (int) (Math.random() * (max - min)) + min;
    return randomNumber;
  }
}
