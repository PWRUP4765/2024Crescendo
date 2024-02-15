package frc.robot.inter;

import java.util.List;

/**
 * @author godbrigero
 * @purpose to make the replay logger subsystem able to be used for many devices
 */
public interface ReplayLoggerDevice {
  /**
   * @return returns the data OR NULL if data is not available
   */
  default double[] getData() {
    return null;
  }

  /**
   * @return returns the data OR NULL if data is not avail
   * @param n cant make two functions with same name so this is useless
   */
  default List<Double> getData(boolean n) {
    return null;
  }
}
