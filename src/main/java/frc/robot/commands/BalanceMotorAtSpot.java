package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.error.LimitException;
import frc.robot.inter.BalanceMotorAtSpotInterface;
import frc.robot.util.MathFun;

/**
 * @author godbrigero
 */
public class BalanceMotorAtSpot extends Command {

  final BalanceMotorAtSpotInterface motorCommands;
  final double motorSpeed, destPos, getMotorCurPosCoef, acuracy, motorMaxSpeedPerc;
  private double last, yCoef;
  private boolean isWorking = false;

  /**
   * @param motorCommands the commands that are passed into the code so the code can control the arm
   * @param normalSpeed the speed at which the arm will be going when it is below / above the desired location
   * @param destPos the destination position of the return of the motorCommands.getMotorPos
   * @param getMotorCurPosCoef if you have a circle, you will need like a conversion factor from revolutions to actual pos
   * this is for that. If your function (getPos) returns the amt of revolutions, you will have to multiply the revolutions by
   * the circum. (ik this is kinda false dont ask)
   * @param acuracy the +- spot that you want the function to be at (0 is not recomended)
   */
  public BalanceMotorAtSpot(
    BalanceMotorAtSpotInterface motorCommands,
    double normalSpeed,
    double destPos,
    double getMotorCurPosCoef,
    double acuracy
  ) {
    this.motorCommands = motorCommands;
    this.motorSpeed = normalSpeed;
    this.destPos = destPos;
    this.getMotorCurPosCoef = getMotorCurPosCoef;
    this.motorMaxSpeedPerc = 0;
    this.acuracy = acuracy;
  }

  /**
   * @param motorCommands the motor commands and outputs
   * @param destPos the destionation position of the motor
   * @param motorMaxSpeedPerc since this is an adaptive method of moving the motor to position, it can have a max speed. In %.
   * @param getMotorCurPosCoef curPosCoef see above
   * @param acuracy +- where the motor should be.
   * @throws LimitException if the motorMaxSpeed is wrong
   */
  public BalanceMotorAtSpot(
    BalanceMotorAtSpotInterface motorCommands,
    double destPos,
    float motorMaxSpeedPerc,
    double getMotorCurPosCoef,
    double acuracy
  ) throws LimitException {
    if (
      motorMaxSpeedPerc > 100 || motorMaxSpeedPerc < 0
    ) throw new LimitException(motorMaxSpeedPerc, this.getClass().getName());
    this.motorCommands = motorCommands;
    this.motorSpeed = -1;
    this.destPos = destPos;
    this.getMotorCurPosCoef = getMotorCurPosCoef;
    this.motorMaxSpeedPerc = motorMaxSpeedPerc;
    this.acuracy = acuracy;
  }

  @Override
  public void initialize() {
    try {
      motorCommands.setMotorSpeed(0);
    } catch (LimitException e) {
      e.printStackTrace();
    }
  }

  @Override
  public void execute() {
    if (!isWorking) return;

    try {
      double p = motorCommands.getMotorCurPosition() * getMotorCurPosCoef;
      if (motorSpeed != -1) {
        motorCommands.setMotorSpeed(p < destPos ? motorSpeed : -motorSpeed);
        return;
      }

      if (last == 0) {
        last = p;
        return;
      }

      // Since the speed of the motor is on the graph, the speed will start very fast and then get slower as time passes
      // we need change because motors have threshhold max speed.
      double change = p - last;
      double x = destPos - p;
      // This if statement adjusts for window
      if (p < destPos - acuracy && p < destPos + acuracy) {
        // YCoef is used to adapt to the motor not having enough power to move up even though it is not in position.
        // Might need re-coding
        yCoef += change <= 0 ? 1 : 0;
        double motorSpeed = MathFun.getPosOnGraph(x, yCoef) * 100;
        if (motorSpeed < motorMaxSpeedPerc) {
          motorCommands.setMotorSpeed(motorSpeed);
        } else {
          // TODO: this might have to be re-coded THIS IS JST FOR DEBUG!
          throw new LimitException(motorSpeed, this.getClass().getName());
        }
      } else if (p > destPos + acuracy && p > destPos - acuracy) {
        yCoef -= change >= 0 ? 1 : 0;
        double motorSpeed = MathFun.getPosOnGraph(x, yCoef) * 100;
        if (motorSpeed < motorMaxSpeedPerc) {
          motorCommands.setMotorSpeed(motorSpeed);
        } else {
          // TODO: this might have to be re-coded THIS IS JST FOR DEBUG!
          throw new LimitException(motorSpeed, this.getClass().getName());
        }
      }
    } catch (LimitException e) {
      e.printStackTrace();
    }
  }

  @Override
  public void end(boolean interrupted) {
    isWorking = !isWorking;
    try {
      motorCommands.setMotorSpeed(0);
    } catch (LimitException e) {
      e.printStackTrace();
    }
  }

  @Override
  public boolean isFinished() {
    return !isWorking;
  }
}
