package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class GoToAprilTag extends CommandBase {

  private final Vision m_vision;
  private Supplier<Double> m_xaxisSpeedSupplier;
  private Supplier<Double> m_zaxisRotateSupplier;
  private Drivetrain m_driveTrain;

  public GoToAprilTag(Drivetrain driveTrain, Vision vision) {
    m_vision = vision;
    m_driveTrain = driveTrain;
    addRequirements(vision);
    addRequirements(driveTrain);
  }

  public GoToAprilTag(
      Drivetrain driveTrain,
      Vision vision,      
      Supplier<Double> xaxisSpeedSupplier,
      Supplier<Double> zaxisRotateSupplier) {
    this(driveTrain, vision);
    m_xaxisSpeedSupplier = xaxisSpeedSupplier;
    m_zaxisRotateSupplier = zaxisRotateSupplier;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Vision-alignment mode
    // Query the latest result from PhotonVision
    var target = m_vision.getBestTarget(); 
    if ( target!=null ) {
      m_xaxisSpeedSupplier = new ValueSupplier(m_vision.getForwardSpeed(target)); 
      m_zaxisRotateSupplier = new ValueSupplier(m_vision.getRotationSpeed(target));
      // Use our forward/turn speeds to control the drivetrain
      m_driveTrain.arcadeDrive(m_xaxisSpeedSupplier.get(), m_zaxisRotateSupplier.get());      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  class ValueSupplier implements Supplier<Double> {
    
    private Double m_value;

    ValueSupplier(Double value) {
      m_value = value;
    }

    public Double get() {
      return m_value;
    }
  }
}