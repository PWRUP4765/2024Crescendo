package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class OutputTimed extends Command {

  private final IntakeSubsystem m_intakeSubsystem;

  private final double outputSpeed = IntakeConstants.kOutputSpeed;
  private int loopCount = 0;
  private final int maxLoops;

  /**
   * @param intakeSubsystem The subsystem used by this command.
   * @param time The time to run the intake, in seconds
   */
  public OutputTimed(IntakeSubsystem intakeSubsystem, double time) {
    this.m_intakeSubsystem = intakeSubsystem;

    maxLoops = (int) (time * 50);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    loopCount = 0;
  }

  @Override
  public void execute() {
    // setting the motor speed to outputSpeed
    m_intakeSubsystem.setMotor(outputSpeed);
    loopCount += 1;
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.setMotor(0);
  }

  @Override
  public boolean isFinished() {
    return loopCount >= maxLoops;
  }
}
