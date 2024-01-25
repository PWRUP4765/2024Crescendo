package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends subsystem{

    private CANSparkMax m_intakeMotor;
    private CANSparkMax m_shooterMotor1;
    private CANSparkMax m_shooterMotor2;
    
    private final m_intakeSensor;

    private final IsBrushless = IntakeConstants.kIntakeIsBrushless ? MotorType.kBrushless : MotorType.kBrushed;

    public void IntakeSubsytem(){

        m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakePort1, IsBrushless);
        m_shooterMotor1 = new CANSparkMax(IntakeConstants.kIntakePort2, IsBrushless);
        m_shooterMotor2 = new CANSparkMax(IntakeConstants.kIntakePort1, IsBrushless;

        //intakeSensor = new...

        m_intakeMotor.setInverted(IntakeConstants.kIsIntakeReversed);
        m_shooterMotor1.setInverted(IntakeConstants.kIsShooter1Reversed);
        m_shooterMotor2.setInverted(IntakeConstants.kIsShooter2Reversed);


    }

    // speed is from [-1 to 1]
    public void setMotor(int speed){ 
            m_intakeMotor.set(speed)
    }

    public boolean isDetected(){
       //return m_intakeSensor.
       return false; 
    }

}
