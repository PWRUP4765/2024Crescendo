package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.error.LimitException;
import frc.robot.subsystems.ClimbArmSubsystem;

/**
 * @apiNote This is just for extending the arm / decending. It does not keep the robot in place for now (when finished)
 * @TODO Add code in order to stay in same spot (relative)
 * @author godbrigero
 * @author Ari
 */

public class ClimbArmCommand extends Command {

  // Assuming that when the gear rotates, the arm moves the diameter

  final double gearDiameter, armLen, speed, circum, gotoPos, acuracy;
  final ClimbArmSubsystem subsystem;
  final boolean stayInPlace;
  boolean isOnline = false;

  /**
   *
   * @param gearDiameter The diameter of the moving gear that is moving the arm IN METERS
   * @param armLen The length of the arm IN METERS
   * @param speed The speed at which you want the motor to excecute the action. Fast = possibly less acurate
   * IN PERCENT (100% max)
   * @param subsystem The arm subsystem with all params setup
   * @param goToPos The Position IN METERS to which you want the arm to go to (arm len - 0.1 = max) i think
   * @param acuracy Since the motor will adjust it's position based on where it is right now, the acurasy
   * is like how accurate you want the motor to go to a specific position. For example if u put 1 then the motor will
   * stop when it is in the window of a +- meter of the goToPos
   * @param stayInPlace this will make the arm stay in place untill told not to AKA interrupted / stopped / canceled
   * @throws LimitException if the goToPos is greater then the length of the arm, it is impossible to go to it.
   * Similarly if the speed exceeds 100% / 0, the exception will be thrown
   *
   */

  public ClimbArmCommand(
    double gearDiameter,
    double armLen,
    double speed,
    ClimbArmSubsystem subsystem,
    double goToPos,
    double acuracy,
    boolean stayInPlace
  ) throws LimitException {
    // check if any limits have been exceeded and throw a new exception if they have been.
    if (
      goToPos > armLen || goToPos < 0 || speed < 0 || speed > 100
    ) throw new LimitException(goToPos, this.getClass().getName());

    this.gearDiameter = gearDiameter;
    this.armLen = armLen;
    this.speed = speed;
    this.subsystem = subsystem;
    this.isOnline = true;
    this.circum = gearDiameter * Math.PI;
    this.gotoPos = goToPos;
    this.acuracy = acuracy;
    this.stayInPlace = stayInPlace;
  }

  @Override
  public void initialize() {
    // TODO: add @this possibly
  }

  @Override
  public void execute() {
    // convert from revs to dist
    double revs = subsystem.getRevSinceStart();
    double curDist = circum * revs;

    if (Math.abs(curDist - gotoPos) < acuracy && !stayInPlace) {
      isOnline = false;
      return;
    }

    try {
      // double if statement look terrible
      subsystem.setSpeed(curDist < gotoPos ? speed : -speed);
    } catch (LimitException e) {
      isOnline = false;
      e.printStackTrace();
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.isOnline = !interrupted;
    subsystem.stopMotor();
  }

  @Override
  public boolean isFinished() {
    return !isOnline;
  }
}
