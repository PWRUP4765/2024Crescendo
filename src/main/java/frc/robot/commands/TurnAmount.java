package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.MathFunc;

public class TurnAmount extends Command {
    
    private final SwerveSubsystem m_swerveSubsystem;
    private final PIDController m_turnPIDController = new PIDController(1.2, 0, 0.2);

    private double desiredAngle;
    private final double angleChange;
    

    /**
     * 
     * @param swerveSubsystem 
     * @param angle measured in [-0.5, 0.5]
     */
    public TurnAmount(SwerveSubsystem swerveSubsystem, double angle) {
        m_swerveSubsystem = swerveSubsystem;

        m_turnPIDController.enableContinuousInput(-0.5, 0.5);

        angleChange = angle;
    }

    @Override
    public void initialize() {
        desiredAngle = m_swerveSubsystem.getGyroAngle() + angleChange;
        desiredAngle = MathFunc.putWithinHalfToHalf(desiredAngle);
    }

    @Override
    public void execute() {
        double currentAngle = m_swerveSubsystem.getGyroAngle();
        double rotationSpeed = m_turnPIDController.calculate(currentAngle, desiredAngle);

        m_swerveSubsystem.drive(0, 0, rotationSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveSubsystem.drive(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_swerveSubsystem.getGyroAngle() - desiredAngle) < 0.008
        || Math.abs(m_swerveSubsystem.getGyroAngle() - desiredAngle) > 359.992;
    }
}
