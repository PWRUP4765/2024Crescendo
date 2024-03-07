package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class Leave extends Command {

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private SwerveSubsystem m_drive;

  private double m_speed;
  private double m_seconds;
  private double m_iterations;
  private double m_counter;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Leave(SwerveSubsystem subsystem, double speed, double seconds) {
    m_drive = subsystem;
    m_speed = speed;
    m_seconds = seconds;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_counter = 0;
    m_iterations = m_seconds * 50;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive(m_speed, 0, 0);
    m_counter += 1;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_counter >= m_iterations) {
      return true;
    } else {
      return false;
    }
  }
}
