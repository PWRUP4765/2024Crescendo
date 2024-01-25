package frc.robot.util;

/**
 * @name MathFun = Math Functions
 */
public class MathFun {

  /**
   * @param x the X position on the graph
   * @param yCoef the the height of the graph on the yAxis. This will increase / decrease the return val
   * @return The Y of the X on the graph
   */
  public static double getPosOnGraph(double x, double yCoef) {
    return Math.cbrt(x) + yCoef;
  }
}
