package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.error.LimitException;
import frc.robot.inter.BalanceMotorAtSpotInterface;
import frc.robot.util.MathFun;

/**
 * @author godbrigero
 * @apiNote this features both an adaptive method of having the arm stay / move to a place and a avg method which idk if works
 */
public class BalanceMotorAtSpot extends Command {

  final BalanceMotorAtSpotInterface motorCommands;
  final boolean stayAtTop;
  final double motorSpeed, destPos, getMotorCurPosCoef, motorMaxSpeedPerc, acuracy;
  double topStaySpeed = 101;
  long time;
  private boolean isWorking = false;

  /**
   * @param motorCommands the motor commands
   * @param normalSpeed the speed at which the motor will execute
   * @param destPos the destination pos
   * @param getMotorCurPosCoef since the "getPosition()" ofter returns its value in revolutions, we need to convert that into
   * meters. This is what this value is for. The code will multiply the "getPosition()" by this value
   * @param acuracy the +- position that the motor has to be in
   * @param stayAtTop if true, it will TRY to stay on top. If the normal speed istoo high tho, it will just break (probably)
   */
  public BalanceMotorAtSpot(
    BalanceMotorAtSpotInterface motorCommands,
    double normalSpeed,
    double destPos,
    double getMotorCurPosCoef,
    double acuracy,
    boolean stayAtTop
  ) {
    this.motorCommands = motorCommands;
    this.motorSpeed = normalSpeed;
    this.destPos = destPos;
    this.getMotorCurPosCoef = getMotorCurPosCoef;
    this.motorMaxSpeedPerc = 0;
    this.time = 0;
    this.stayAtTop = stayAtTop;
    this.acuracy = 0;
  }

  /**
   * @param motorCommands the motor comands such as "setSpeed()"
   * @param destPos the destination position in Meters
   * @param getMotorCurPosCoef since the "getPosition()" ofter returns its value in revolutions, we need to convert that into
   * meters. This is what this value is for. The code will multiply the "getPosition()" by this value
   * @param time the time it will take the arm to get to the spot
   * @param stayAtTop if true will stay at top. if not, will just turn off the motor
   */
  public BalanceMotorAtSpot(
    BalanceMotorAtSpotInterface motorCommands,
    double destPos,
    double getMotorCurPosCoef,
    long time,
    boolean stayAtTop
  ) {
    this.motorCommands = motorCommands;
    this.motorSpeed = -1;
    this.destPos = destPos;
    this.getMotorCurPosCoef = getMotorCurPosCoef;
    this.motorMaxSpeedPerc = 0;
    this.time = time;
    this.stayAtTop = stayAtTop;
    this.acuracy = 0;
  }

  @Override
  public void initialize() {
    try {
      motorCommands.setMotorSpeed(100);
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

        if (!stayAtTop && Math.abs(Math.abs(p) - Math.abs(destPos)) < acuracy) {
          isWorking = false;
        }

        return;
      }

      if (topStaySpeed != 101) {
        if (stayAtTop) {
          motorCommands.setMotorSpeed(topStaySpeed);
          return;
        } else {
          this.isWorking = false;
        }
      }

      double curMotorSpeed = motorCommands.getCurMotorSpeedPerc();
      boolean isDown = destPos - p < 0;

      double neededChange = getAccelerationChangeNeeded(
        time,
        isDown ? p : destPos,
        isDown ? destPos : p,
        curMotorSpeed
      );

      if (
        motorMaxSpeedPerc > curMotorSpeed && -motorMaxSpeedPerc < curMotorSpeed
      ) {
        motorCommands.setMotorSpeed(curMotorSpeed - neededChange);
      } else {
        throw new LimitException(curMotorSpeed, this.getClass().getName());
      }

      time -= 0.02;

      if (time <= 0) {
        // TODO: this MIGHT need to be re-coded. Because the top speed might not be the best speed that makes the thing stay
        // in one place ngl this needs testing.
        topStaySpeed = curMotorSpeed;
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

  /**
   * @param time time to excecute
   * @param positionEnd end position of the "throw" in Meters
   * @param positionStart start pos (ex - 0) in Meters
   * @param currentSpeedPerc current speed
   * @return will return the number by which you will have to slow down ur sleed (-=) for the motor to reach the target
   */
  private double getAccelerationChangeNeeded(
    long time,
    double positionEnd,
    double positionStart,
    double currentSpeedPerc
  ) {
    return (
      (2 * positionEnd - positionStart - currentSpeedPerc * time) / time * time
    );
  }
}
