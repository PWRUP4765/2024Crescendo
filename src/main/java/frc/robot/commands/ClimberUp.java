package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.error.LimitException;
import frc.robot.inter.ArmInterface;
import frc.robot.subsystems.ClimbArmSubsystem;

public class ClimberUp extends Command {

  private final ClimbArmSubsystem sub;
  private final double speed;
  private final ArmInterface armInterface;

  public ClimberUp(
    ClimbArmSubsystem sub,
    double speed,
    ArmInterface armInterface
  ) {
    this.armInterface = armInterface;
    this.sub = sub;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    armInterface.lockArm();
  }

  @Override
  public void execute() {
    try {
      sub.setSpeed(armInterface.getCurArmPosition() <= 0.02 ? speed : 0);
    } catch (LimitException e) {
      e.printStackTrace();
    }
  }

  @Override
  public void end(boolean interrupted) {
    try {
      sub.setSpeed(0);
    } catch (LimitException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
