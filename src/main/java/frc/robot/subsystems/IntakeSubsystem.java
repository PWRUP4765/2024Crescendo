package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
    //PLACEHOLDER
    private CANSparkMax intakeSensor;
    private CANSparkMax inputMotor;
    private CANSparkMax shooterMotor1;
    private CANSparkMax shooterMotor2;

    public void IntakeSubsytem(int inputMotorChannel, int sensorChannel, int shooterMotorChannel, int shooterMotorChannel2){
        //PLACEHOLDER
        intakeSensor = new CANSparkMax(sensorChannel, MotorType.kBrushed);
        inputMotor = new CANSparkMax(inputMotorChannel, MotorType.kBrushless);
        shooterMotor1 = new CANSparkMax(shooterMotorChannel, MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(shooterMotorChannel2, MotorType.kBrushless);

    }
    @Override
    public void periodic() {
        // Check if the intake sensor is triggered
        if (/*intakeSensor.get()*/ true) {
            // Start intake motor
            inputMotor.set(1.0); // Adjust speed as necessary
        } else {
            // Stop intake motor
            inputMotor.set(0.0);
        }

        // Check if the shooter sensor is triggered
        if (/*intakeSensor.get()*/ true) {
            // Start shooter motors
            shooterMotor1.set(1.0); // Adjust speed as necessary
            shooterMotor2.set(1.0); // Adjust speed as necessary
        } else {
            // Stop shooter motors
            shooterMotor1.set(0.0);
            shooterMotor2.set(0.0);
        }
    }

}
