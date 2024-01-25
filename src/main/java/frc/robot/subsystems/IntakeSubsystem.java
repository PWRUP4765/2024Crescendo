package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax m_intakeMotor;
  private CANSparkMax m_shooterMotor1;
  private CANSparkMax m_shooterMotor2;

  // idk what this is.
  //private final m_intakeSensor;

  private final MotorType IsBrushless = IntakeConstants.kIsBrushless
    ? MotorType.kBrushless
    : MotorType.kBrushed;

  public void IntakeSubsytem() {
    /*m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakePort1, IsBrushless);
        m_shooterMotor1 = new CANSparkMax(IntakeConstants.kIntakePort2, IsBrushless);
        m_shooterMotor2 = new CANSparkMax(IntakeConstants.kIntakePort1, IsBrushless;*/

    //intakeSensor = new...

    m_intakeMotor.setInverted(IntakeConstants.kIsIntakeReversed);
    m_shooterMotor1.setInverted(IntakeConstants.kIsShooter1Reversed);
    m_shooterMotor2.setInverted(IntakeConstants.kIsShooter2Reversed);
  }

  // speed is from [-1 to 1]
  public void setMotor(int speed) {
    m_intakeMotor.set(speed);
  }

  public boolean isDetected() {
    //return m_intakeSensor.
    return false;
  }
}
