package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.error.LimitException;
import frc.robot.subsystems.ClimbArmSubsystem;

/**
 * @author godbrigero
 */
public class ClimbArmCommand extends Command {

  boolean isOnline = true;
  final ClimbArmSubsystem subsystem;
  final double pos, positionToReturnTo;
  final String suffleboardString;
  final GenericEntry entryPosition, entrySpeed;

  /**
   * @param subsystem the arm subsystem
   * @param position the desired position IN METERS
   * @param positionToReturnTo the position to return to IN METERS
   * @param shuffleboardString the shuffleboard TAB name
   */
  public ClimbArmCommand(
    ClimbArmSubsystem subsystem,
    double position,
    double positionToReturnTo,
    String shuffleboardString
  ) {
    this.subsystem = subsystem;
    this.pos = position;
    this.positionToReturnTo = positionToReturnTo;
    this.suffleboardString = shuffleboardString;

    this.entryPosition =
      Shuffleboard
        .getTab(shuffleboardString)
        .add("Climb Arm Position", 0)
        .getEntry();
    this.entrySpeed =
      Shuffleboard
        .getTab(shuffleboardString)
        .add("Climb Arm Current Speed", 0)
        .getEntry();
  }

  @Override
  public void initialize() {
    try {
      subsystem.setReference(pos);
    } catch (LimitException e) {
      e.printStackTrace();
    }
  }

  @Override
  public void execute() {
    entryPosition.setDouble(subsystem.getRevSinceStart());
    entrySpeed.setDouble(subsystem.getCurrentSetSpeedPerc());
  }

  @Override
  public void end(boolean interrupted) {
    this.isOnline = !interrupted;
    try {
      subsystem.setReference(this.positionToReturnTo);
    } catch (LimitException e) {
      e.printStackTrace();
    }
  }

  @Override
  public boolean isFinished() {
    return !isOnline;
  }
}
