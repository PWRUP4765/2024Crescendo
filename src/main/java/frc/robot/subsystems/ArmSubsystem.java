package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.inter.ArmInterface;

public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax m_armMotor;
  private SparkPIDController m_armPIDController;
  private SparkAbsoluteEncoder m_armEncoder;

  private double currentSetPosition = 0.0, kP, kI, kD, kIZ, kFF;

  private ShuffleboardTab sb_tab;
  private String sb_name;
  private GenericEntry sb_encoderPosition, sb_setPosition;

  private boolean isLocked;

  /**
   * Constructor class for ArmSubsystem
   */
  public ArmSubsystem() {
    //example code for REVrobotics parts: https://github.com/REVrobotics/SPARK-MAX-Examples/tree/master/Java

    //setting up the arm motor
    m_armMotor =
      new CANSparkMax(ArmConstants.kArmMotorPort, MotorType.kBrushed);
    m_armMotor.restoreFactoryDefaults();
    m_armMotor.setInverted(ArmConstants.kArmMotorReversed);

    kP = ArmConstants.kP;
    kI = ArmConstants.kI;
    kD = ArmConstants.kD;
    kIZ = ArmConstants.kIZ;
    kFF = ArmConstants.kFF;

    m_armPIDController = m_armMotor.getPIDController();
    m_armPIDController.setP(kP);
    m_armPIDController.setI(kI);
    m_armPIDController.setD(kD);
    m_armPIDController.setIZone(kIZ);
    m_armPIDController.setFF(kFF);

    m_armEncoder =
      m_armMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    m_armEncoder.setZeroOffset(ArmConstants.kEncoderOffset);
    m_armEncoder.setInverted(ArmConstants.kEncoderReversed);
    m_armEncoder.setPositionConversionFactor(
      ArmConstants.kEncoderConversionFactor
    );
    m_armPIDController.setFeedbackDevice(m_armEncoder);

    m_armMotor.burnFlash();
  }

  /**
   * Doesn't change the set position of the arm, but does change the arbFF of the motor controller based on the encoder position
   */
  public void updateFF() {
    if (currentSetPosition == 0 && m_armEncoder.getPosition() < 0.008) {
      m_armPIDController.setReference(
        currentSetPosition,
        CANSparkBase.ControlType.kPosition,
        0,
        0
      );
      m_armPIDController.setIAccum(0);
    } else {
      m_armPIDController.setReference(
        currentSetPosition,
        CANSparkBase.ControlType.kPosition,
        0,
        ArmConstants.kFFCoefficient *
        Math.cos(m_armEncoder.getPosition() * (2 * Math.PI))
      );
    }
  }

  /**
   * The arm will begin moving to the desired position. 0 means flat forwards, 0.5 means flat backwards.
   * @param Position The position to move to. Domain: [0, 0.25]
   */
  public void setPosition(double position) {
    if (!isLocked) {
      currentSetPosition = position;
    }

    updateFF();
    //sb_setPosition.setDouble(position);
  }

  public double getCurrentPosition() {
    return m_armEncoder.getPosition();
  }

  public Command setPositionCommand(double position) {
    return runOnce(() -> {
      setPosition(position);
    });
  }

  public void createShuffleboardTab() {
    sb_name = "ArmSubsystem";
    sb_tab = Shuffleboard.getTab(sb_name);

    sb_encoderPosition = sb_tab.add("encoderPosition", 0).getEntry();
    sb_setPosition = sb_tab.add("setPosition", 0).getEntry();
  }

  public void updateShuffleboardTab() {
    sb_encoderPosition.setDouble(m_armEncoder.getPosition());
  }

  /**
   *
   * @param isLocked the boolean of if arm is locked or not
   */
  public void setLocked(boolean isLocked) {
    this.isLocked = isLocked;
  }

  /**
   *
   * @param isLocked the boolean of if arm is locked or not
   * @param positionLocked the position at which the arm will end up (will go to here even though locked then stay there)
   */
  public void setLocked(boolean isLocked, double positionLocked) {
    setPosition(positionLocked);
    this.isLocked = isLocked;
  }

  public ArmInterface getArmInterface() {
    return new ArmInterface() {
      @Override
      public double getCurArmPosition() {
        return getCurrentPosition();
      }

      @Override
      public void lockArm() {
        setLocked(true, Constants.ArmConstants.kArmFlatPosition);
      }

      @Override
      public void unlockArm() {
        setLocked(false);
      }
    };
  }
}
