package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX; 


/**
 * @author seankusu
 */

public class IntakeSubsystem extends SubsystemBase{

    private TalonFX m_intakeMotor;
    private CANSparkMax m_shooterMotor1;
    private CANSparkMax m_shooterMotor2;
    
    // This is TBD depedning on what sensor hardware decides to use
    //private final m_intakeSensor;

    private final MotorType IsBrushless = IntakeConstants.kIsBrushless ? MotorType.kBrushless : MotorType.kBrushed;

    public void IntakeSubsytem(){

        m_intakeMotor = new TalonFX(IntakeConstants.kIntakePort, IntakeConstants.kCanbusAddress);
        m_shooterMotor1 = new CANSparkMax(IntakeConstants.kShooterPort1, IsBrushless);
        m_shooterMotor2 = new CANSparkMax(IntakeConstants.kShooterPort2, IsBrushless);

        m_intakeMotor.setInverted(IntakeConstants.kIsIntakeReversed);
        m_shooterMotor1.setInverted(IntakeConstants.kIsShooter1Reversed);
        m_shooterMotor2.setInverted(IntakeConstants.kIsShooter2Reversed);

        //intakeSensor = new...
    }

    /**
     * @param speed will take a value between [-1, 1]
     */
    public void setMotor(int speed){ 
            m_intakeMotor.set(speed);
    }

    /** 
     * @return returns whether the sensor detects an object 
     */
    public boolean isDetected(){
       //return m_intakeSensor.
       return false; 
    }

}
