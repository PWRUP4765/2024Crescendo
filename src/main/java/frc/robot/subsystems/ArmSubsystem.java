package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.ArmConstants;


public class ArmSubsystem {
    
    private CANSparkMax armMotor;
    private SparkPIDController armPIDController;
    private SparkAbsoluteEncoder armEncoder;

    private double
    kP,
    kI,
    kD,
    kIZ,
    kFF;

    private ShuffleboardTab sb_tab;
    private String sb_name;

    private GenericEntry
    sb_kP,
    sb_kI,
    sb_kD,
    sb_kIZ,
    sb_kFF,

    sb_encoderPosition;

    /**
     * Constructor class for ArmSubsystem
     */
    public ArmSubsystem() {
        //example code for REVrobotics parts: https://github.com/REVrobotics/SPARK-MAX-Examples/tree/master/Java

        //setting up the arm motor
        armMotor = new CANSparkMax(ArmConstants.kArmMotorPort, MotorType.kBrushless);
        armMotor.setInverted(ArmConstants.kArmMotorReversed);

        kP = ArmConstants.kArmP;
        kI = ArmConstants.kArmI;
        kD = ArmConstants.kArmD;
        kIZ = ArmConstants.kArmIZ;
        kFF = ArmConstants.kArmFF;

        armPIDController = armMotor.getPIDController();
        armPIDController.setP(kP);
        armPIDController.setI(kI);
        armPIDController.setD(kD);
        armPIDController.setIZone(kIZ);
        armPIDController.setFF(kFF);
        armPIDController.setOutputRange(ArmConstants.kArmMinOutput, ArmConstants.kArmMaxOutput);

        armEncoder = armMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        armEncoder.setZeroOffset(ArmConstants.kArmEncoderOffset);
        armEncoder.setPositionConversionFactor(ArmConstants.kArmEncoderConversionFactor);
        armPIDController.setFeedbackDevice(armEncoder);
    }

    /**
     * The arm will begin moving to the desired position. 0 means flat forwards, 1 means flat backwards.
     * @param Position The position to move to. Domain: [0, 1]
     */
    public void setPosition(double position) {


    }

    public void createShuffleboardTab() {
        sb_name = "ArmSubsystem";
        sb_tab = Shuffleboard.getTab(sb_name);

        sb_kP = sb_tab.add("kP", kP).getEntry();
        sb_kI = sb_tab.add("kI", kI).getEntry();
        sb_kD = sb_tab.add("kD", kD).getEntry();
        sb_kIZ = sb_tab.add("kIZ", kIZ).getEntry();
        sb_kFF = sb_tab.add("kFF", kFF).getEntry();

        sb_encoderPosition = sb_tab.add("sb_encoderPosition", 0).getEntry();
    }

    public void updateShuffleboardTab() {
        sb_encoderPosition.setDouble(armEncoder.getPosition());
    }

}
