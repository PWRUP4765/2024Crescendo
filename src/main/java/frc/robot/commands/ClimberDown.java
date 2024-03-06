package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.error.LimitException;
import frc.robot.inter.ArmInterface;
import frc.robot.subsystems.ClimbArmSubsystem;

public class ClimberDown extends Command {

  private final ClimbArmSubsystem sub;
  private final double speed;
  private final ArmInterface armInterface;
  private boolean isOnline;

  public ClimberDown(
    ClimbArmSubsystem sub,
    double speed,
    ArmInterface armInterface
  ) {
    this.armInterface = armInterface;
    this.sub = sub;
    this.speed = speed;
  }

  @Override
  public void execute() {
    try {
      sub.setSpeed(-speed);
    } catch (LimitException e) {
      e.printStackTrace();
    }
  }

  @Override
  public void end(boolean interrupted) {
    isOnline = false;
    if (sub.isArmOnBottom()) armInterface.unlockArm();
  }

  @Override
  public boolean isFinished() {
    return !isOnline;
  }
}
