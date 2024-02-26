package frc.robot.subsystems;

import java.lang.reflect.Array;

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
    private IntakeSensor m_sensor;

    private final MotorType IsBrushless = IntakeConstants.kIsBrushless
        ? MotorType.kBrushless
        : MotorType.kBrushed;

    public IntakeSubsystem(){
        m_intakeMotor = new PWMTalonSRX(9);
        m_intakeMotor.setInverted(IntakeConstants.kIsIntakeReversed);

        m_sensor = new IntakeSensor();
    }

    /**
     * @param speed will take a value between [-1, 1]
     */
    public void setMotor(double speed) {
        m_intakeMotor.set(speed);
    }

    public int[] getSensorValue() {
        return m_sensor.getSensorValue();
    }

    /** 
     * @return returns whether the sensor detects an object 
     */
    public boolean isDetected(){
       return m_sensor.isDetected();
    }

    public class IntakeSensor { 

        private AnalogInput m_analog1, m_analog2, m_analog3, m_analog4;

        public IntakeSensor() {
            m_analog1 = new AnalogInput(0);
            m_analog2 = new AnalogInput(1);
            m_analog3 = new AnalogInput(2);
            m_analog4 = new AnalogInput(3);
        }

        /**
         * @return The average values of all the sensors in an integer array
         */
        public int[] getSensorValue(){
            int[] AverageValues = {m_analog1.getAverageValue(), m_analog2.getAverageValue(), m_analog3.getAverageValue(), m_analog4.getAverageValue()};
            return AverageValues;
        }

        /**
         * @return Whether any of the four analog inputs detects something in the intake
         */
        public boolean isDetected(){ 
            int[] AverageValues = getSensorValue();

            for (int analogValue : AverageValues){
                if (analogValue < 2500) { return true; }
            }
            return false;
        }
    }


}

