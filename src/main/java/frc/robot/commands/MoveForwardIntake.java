package frc.robot.commands;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSensor;

public class MoveForwardIntake extends Command {
    
    private final SwerveSubsystem m_swerveSubsystem;
    private final ArmSubsystem m_armSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;
    

    public MoveForwardIntake(
        SwerveSubsystem swerveSubsystem,
        ArmSubsystem armSubsystem,
        IntakeSubsystem intakeSubsystem
    ) {
        m_swerveSubsystem = swerveSubsystem;
        m_armSubsystem = armSubsystem;
        m_intakeSubsystem = intakeSubsystem;

        addRequirements(swerveSubsystem, intakeSubsystem, armSubsystem);
    }

    @Override
    public void initialize() {
        m_armSubsystem.setPosition(ArmConstants.kArmFlatPosition);
    }

    @Override
    public void execute() {
        if (m_armSubsystem.getCurrentPosition() >= 0.1) {
            return;
        }
        m_swerveSubsystem.drive(0, -0.2, 0);
        m_intakeSubsystem.setMotor(IntakeConstants.kIntakeSpeed);
        if (m_intakeSubsystem.getIntakeCurrent() > IntakeConstants.kIntakeCurrentThresholdAmps) {
            m_armSubsystem.setPosition(ArmConstants.kArmDrivingPosition);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveSubsystem.drive(0, 0, 0);
        m_intakeSubsystem.setMotor(0);
        m_armSubsystem.setPosition(ArmConstants.kArmDrivingPosition);
    }

    @Override
    public boolean isFinished() {
        return m_intakeSubsystem.isDetected();
    }
}
