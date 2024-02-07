package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

/**
 * @author seankusu
 */

public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax m_intakeMotor;
    private CANSparkMax m_shooterMotor1;
    private CANSparkMax m_shooterMotor2;
    private AnalogInput m_analog;
    
    private double IntakeSpeed = IntakeConstants.kIntakeSpeed;
    
    // This is TBD depedning on what sensor hardware decides to use
    //private final m_intakeSensor;

  // This is TBD depedning on what sensor hardware decides to use
  //private final m_intakeSensor;

  private final MotorType IsBrushless = IntakeConstants.kIsBrushless
    ? MotorType.kBrushless
    : MotorType.kBrushed;

    public IntakeSubsystem(){
        System.out.println("IntakeSubsystem!");

        m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakePort, IsBrushless);

        m_intakeMotor.setInverted(IntakeConstants.kIsIntakeReversed);

        m_analog = new AnalogInput(0);
    }

    /**
     * @param speed will take a value between [-1, 1]
     */
    public void setMotor(int speed) {
        m_intakeMotor.set(speed);
    }

    /**
     * @param speed will take a value between [-1, 1]
     */
    public void setMotor(){
        if (!isDetected()) {
            //System.out.println(isDetected());
            m_intakeMotor.set(IntakeSpeed);
        }
        else {
            IntakeSpeed = 0;
            m_intakeMotor.set(IntakeSpeed);
        }
    }

    public int getSensorValue() {
        /*if (m_analog == null)
        {
            System.out.println("IntakeSubsystem::getSensorValue: new AnalogInput(0)");
            m_analog = new AnalogInput(0);
        } */
        int value = m_analog.getAverageValue();
        System.out.println(value);
        return value;
    }

    /** 
     * @return returns whether the sensor detects an object 
     */
    public boolean isDetected(){
       if (m_analog.getAverageValue() < 2500) {
            System.out.println(true);
            return true;
       }
       else {
        System.out.println(false);
        return false;
       }
    }
}
