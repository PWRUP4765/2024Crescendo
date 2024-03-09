package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveDirectionTimed extends Command {
    
    private final SwerveSubsystem m_swerveSubsystem;

    private int loopCount = 0;
    private final int maxLoops;

    private final double xSpeed, ySpeed;

    public MoveDirectionTimed(
        SwerveSubsystem swerveSubsystem,
        double xSpeed,
        double ySpeed,
        double time
    ) {
        m_swerveSubsystem = swerveSubsystem;

        maxLoops = (int) (time / 20);

        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
    }

    @Override
    public void initialize() {
        loopCount = 0;
    }

    @Override
    public void execute() {
        m_swerveSubsystem.drive(xSpeed, ySpeed, 0, SwerveConstants.kAutonSpeedMultiplier);

        loopCount++;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveSubsystem.drive(0, 0, 0, SwerveConstants.kAutonSpeedMultiplier);
    }

    @Override
    public boolean isFinished() {
        return loopCount >= maxLoops;
    }
}
