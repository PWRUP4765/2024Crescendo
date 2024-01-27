package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.error.LimitException;
import frc.robot.inter.BalanceMotorAtSpotInterface;
import frc.robot.subsystems.ClimbArmSubsystem;

/**
 * @author godbrigero
 */

public class ClimbArmCommand extends Command {

  // Assuming that when the gear rotates, the arm moves the diameter

  final String name = "ClimbArm";
  final double gearDiameter, armLen, circum, gotoPos;
  final ShuffleboardTab sb_tab;
  final boolean stayInPlace;
  final ClimbArmSubsystem subsystem;
  boolean isOnline = false;
  MoveMotor armCommand = null;

  /**
   *
   * @param gearDiameter The diameter of the moving gear that is moving the arm IN METERS
   * @param armLen The length of the arm IN METERS
   * @param goToPos The Position IN METERS to which you want the arm to go to (arm len - 0.1 = max) i think
   * @param stayInPlace this will make the arm stay in place untill told not to AKA interrupted / stopped / canceled
   * @throws LimitException if the goToPos is greater then the length of the arm, it is impossible to go to it.
   *
   */

  public ClimbArmCommand(
    double gearDiameter,
    double armLen,
    double goToPos,
    boolean stayInPlace,
    ClimbArmSubsystem subsystem
  ) throws LimitException {
    // check if any limits have been exceeded and throw a new exception if they have been.
    if (goToPos > armLen || goToPos < 0) throw new LimitException(
      goToPos,
      this.getClass().getName()
    );

    this.gearDiameter = gearDiameter;
    this.armLen = armLen;
    this.isOnline = true;
    this.circum = gearDiameter * Math.PI;
    this.gotoPos = goToPos;
    this.stayInPlace = stayInPlace;
    this.subsystem = subsystem;

    this.sb_tab = Shuffleboard.getTab(name);
  }

  @Override
  public void execute() {
    // FIXME: This will need to be tested
    sb_tab.add("Position Of Climb Arm", 0);
  }

  @Override
  public void initialize() {
    armCommand =
      new MoveMotor(
        new BalanceMotorAtSpotInterface() {
          @Override
          public double getMotorCurPosition() {
            return subsystem.getRevSinceStart();
          }

          @Override
          public void setMotorSpeed(double speedPerc) throws LimitException {
            subsystem.setSpeed(speedPerc);
          }

          @Override
          public double getCurMotorSpeedPerc() {
            return subsystem.getCurrentSetSpeedPerc();
          }
        },
        gotoPos,
        circum,
        10,
        stayInPlace
      );

    CommandScheduler.getInstance().schedule(armCommand);
  }

  @Override
  public void end(boolean interrupted) {
    armCommand.end(true);
    isOnline = false;
  }

  @Override
  public boolean isFinished() {
    return !isOnline;
  }
}
