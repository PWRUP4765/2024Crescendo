package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.error.LimitException;
import frc.robot.subsystems.ClimbArmSubsystem;

public class ClimbArmCommand extends Command {

  boolean isOnline = true;
  final ClimbArmSubsystem subsystem;
  final double pos;

  public ClimbArmCommand(ClimbArmSubsystem subsystem, double position) {
    this.subsystem = subsystem;
    this.pos = position;
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
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    this.isOnline = !interrupted;
    try {
      subsystem.setReference(0);
    } catch (LimitException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  @Override
  public boolean isFinished() {
    return !isOnline;
  }
}
