package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private PWMTalonSRX m_intakeMotor;
    private AnalogInput m_analog;
    
    private double IntakeSpeed = IntakeConstants.kIntakeSpeed;

    private final MotorType IsBrushless = IntakeConstants.kIsBrushless
        ? MotorType.kBrushless
        : MotorType.kBrushed;

    public IntakeSubsystem(){
        m_intakeMotor = new PWMTalonSRX(9);
        m_intakeMotor.setInverted(IntakeConstants.kIsIntakeReversed);
        
        
        
        
        m_analog = new AnalogInput(0);
    }

    /**
     * @param speed will take a value between [-1, 1]
     */
    public void setMotor(double speed) {

        m_intakeMotor.set(speed);
    }

    public int getSensorValue() {
        return m_analog.getAverageValue();
    }

    /** 
     * @return returns whether the sensor detects an object 
     */
    public boolean isDetected(){
       return m_analog.getAverageValue() < 2500 ? true : false; 
    }

    
    public void setMotor(){
        if (!isDetected()) {
            m_intakeMotor.set(IntakeSpeed);
        }
        else {
            IntakeSpeed = 0;
            m_intakeMotor.set(IntakeSpeed);
        }
    }
}
