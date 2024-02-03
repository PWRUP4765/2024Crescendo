package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX; 


/**
 * @author seankusu
 */

public class IntakeSubsystem extends SubsystemBase{

    private TalonFX m_intakeMotor;
    private CANSparkMax m_shooterMotor1;
    private CANSparkMax m_shooterMotor2;
    private AnalogInput m_analog;
    
    // This is TBD depedning on what sensor hardware decides to use
    //private final m_intakeSensor;

    private final MotorType IsBrushless = IntakeConstants.kIsBrushless ? MotorType.kBrushless : MotorType.kBrushed;

    public void IntakeSubsytem(){
        System.out.println("I juts created the m_analog!");
        m_intakeMotor = new TalonFX(IntakeConstants.kIntakePort, IntakeConstants.kCanbusAddress);
        m_shooterMotor1 = new CANSparkMax(IntakeConstants.kShooterPort1, IsBrushless);
        m_shooterMotor2 = new CANSparkMax(IntakeConstants.kShooterPort2, IsBrushless);

        m_intakeMotor.setInverted(IntakeConstants.kIsIntakeReversed);
        m_shooterMotor1.setInverted(IntakeConstants.kIsShooter1Reversed);
        m_shooterMotor2.setInverted(IntakeConstants.kIsShooter2Reversed);

        //intakeSensor = new...
        m_analog = new AnalogInput(0);
        System.out.println("I juts created the m_analog!");
    }

    /**
     * @param speed will take a value between [-1, 1]
     */
    public void setMotor(int speed){ 
        m_intakeMotor.set(speed);
    }
    public int getSensorValue() {
        int value = m_analog.getAverageValue();
        System.out.println(value);
        return value;
    }
    /** 
     * @return returns whether the sensor detects an object 
     */
    public boolean isDetected(){
       //return m_intakeSensor.
       return false; 
    }

}
