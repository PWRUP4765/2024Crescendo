package frc.robot.extern_libs.lidar;

import ev3dev.sensors.slamtec.RPLidarA1;
import ev3dev.sensors.slamtec.RPLidarA1ServiceException;
import ev3dev.sensors.slamtec.RPLidarProviderListener;
import ev3dev.sensors.slamtec.model.Scan;
import java.util.HashMap;

/**
 * @author godbrigero
 * @purpose for now this has no use... but later...
 */
public class RPLidar {

  final RPLidarA1 lidar;
  final HashMap<Long, Scan> args = new HashMap<>();

  public RPLidar(String connectonPort) throws RPLidarA1ServiceException {
    this.lidar = new RPLidarA1(connectonPort);
    lidar.init();

    lidar.addListener(
      new RPLidarProviderListener() {
        @Override
        public void scanFinished(Scan arg0) {
          args.put(System.currentTimeMillis(), arg0);
        }
      }
    );
  }

  /**
   * @throws RPLidarA1ServiceException
   */
  public void startLidar() throws RPLidarA1ServiceException {
    lidar.scan();
  }

  /**
   * @throws RPLidarA1ServiceException
   */
  public void stopLidar() throws RPLidarA1ServiceException {
    lidar.close();
  }

  /**
   * @return if no scans have income since the last scan get, will return null. Otherwise it will return a map of MS and scans since last req of scan get.
   */
  public HashMap<Long, Scan> getLastScans() {
    HashMap<Long, Scan> tmp = args;
    args.clear();
    return tmp;
  }
}
