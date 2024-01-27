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
    /* HIGH LEVEL WHAT WE WANT TO DO:
     * 
     * When the disc is slides in to the robot from the bottom, the intake motors are moving,
     * then once the disc hits a metal sensor, the intake motors stop, and the disc is ready to be shot.
     * Once the command is given to shoot the disc, the intake motors will spin inwards, and the shooter
     * motors will spin clockwise
     */
    @Override
    public void periodic() {

        if (/* controller button pressed */ true) {
            inputMotor.set(1.0);
            shooterMotor1.set(1.0); 
            shooterMotor2.set(1.0); 
        }
        if (/*intakeSensor.get()*/ true) {
            inputMotor.set(0);
            shooterMotor1.set(0); 
            shooterMotor2.set(0); 
        }
    }
}
