package frc.robot.subsystems;

//example phoenix6 code here: https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/CANcoder/src/main/java/frc/robot/Robot.java
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.SwerveConstants;

public class SwerveModule {

  //the driving electronics
  private CANSparkMax driveMotor;
  private SparkPIDController drivePIDController;

  //the turning electronics
  private CANSparkMax turnMotor;
  private SparkPIDController turnPIDController;
  private RelativeEncoder turnRelativeEncoder;

  private CANcoder turnCANcoder;

  //the Shuffleboard tab and entries
  private String sb_abbreviation;
  private ShuffleboardTab sb_tab;

  public double kDriveP, kDriveI, kDriveD, kDriveIZ, kDriveFF;

  public double kTurnP, kTurnI, kTurnD, kTurnIZ, kTurnFF;

  public GenericEntry sb_kDriveP, sb_kDriveI, sb_kDriveD, sb_kDriveIZ, sb_kDriveFF, sb_kTurnP, sb_kTurnI, sb_kTurnD, sb_kTurnIZ, sb_kTurnFF, sb_speed, sb_angle, sb_turnRelativeEncoderAngle, sb_turnCANcoderAngle;

  public SwerveModule(
    int driveMotorChannel,
    boolean driveMotorReversed,
    int turnMotorChannel,
    boolean turnMotorReversed,
    int CANCoderEncoderChannel,
    SensorDirectionValue CANCoderDirection,
    double CANCoderMagnetOffset,
    String abbreviation
  ) {
    //setting up the drive motor controller
    driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    drivePIDController = driveMotor.getPIDController();

    //setting up the turning motor controller and encoders
    turnMotor = new CANSparkMax(turnMotorChannel, MotorType.kBrushless);
    turnPIDController = turnMotor.getPIDController();
    turnRelativeEncoder = turnMotor.getEncoder();

    //setting up the CANCoder
    turnCANcoder = new CANcoder(CANCoderEncoderChannel);

    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.MagnetOffset = -CANCoderMagnetOffset;
    config.MagnetSensor.AbsoluteSensorRange =
      AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    config.MagnetSensor.SensorDirection = CANCoderDirection;

    turnCANcoder.getConfigurator().apply(config);

    //for a full list of SparkMax commands, vist https://robotpy.readthedocs.io/projects/rev/en/latest/rev/RelativeEncoder.html

    //creating the PID values from the SwerveConstants file
    kDriveP = SwerveConstants.kDriveP;
    kDriveI = SwerveConstants.kDriveI;
    kDriveD = SwerveConstants.kDriveD;
    kDriveIZ = SwerveConstants.kDriveIZ;
    kDriveFF = SwerveConstants.kDriveFF;

    kTurnP = SwerveConstants.kTurnP;
    kTurnI = SwerveConstants.kTurnI;
    kTurnD = SwerveConstants.kTurnD;
    kTurnIZ = SwerveConstants.kTurnIZ;
    kTurnFF = SwerveConstants.kTurnFF;

    //setting up the drive motor
    driveMotor.restoreFactoryDefaults();
    driveMotor.setInverted(driveMotorReversed);
    drivePIDController.setP(kDriveP);
    drivePIDController.setI(kDriveI);
    drivePIDController.setD(kDriveD);
    drivePIDController.setIZone(kDriveIZ);
    drivePIDController.setFF(kDriveFF);
    drivePIDController.setOutputRange(
      SwerveConstants.kDriveMinOutput,
      SwerveConstants.kDriveMaxOutput
    );

    //setting up the turn motor
    turnMotor.restoreFactoryDefaults();
    turnMotor.setInverted(turnMotorReversed);
    turnMotor.setSmartCurrentLimit(40);
    turnPIDController.setP(kTurnP);
    turnPIDController.setI(kTurnI);
    turnPIDController.setD(kTurnD);
    turnPIDController.setIZone(kTurnIZ);
    turnPIDController.setFF(kTurnFF);
    turnPIDController.setPositionPIDWrappingEnabled(true);
    turnPIDController.setPositionPIDWrappingMinInput(-1.0 / 2.0);
    turnPIDController.setPositionPIDWrappingMaxInput(1.0 / 2.0);
    turnPIDController.setOutputRange(
      SwerveConstants.kTurnMinOutput,
      SwerveConstants.kTurnMaxOutput
    );
    turnRelativeEncoder.setPosition(
      turnCANcoder.getAbsolutePosition().getValueAsDouble() /
      SwerveConstants.kTurnConversionFactor
    );
    turnRelativeEncoder.setPositionConversionFactor(
      SwerveConstants.kTurnConversionFactor
    );

    //custom function to set up the Shuffleboard tab
    createShuffleboardTab(abbreviation);
  }

  /**
   * Sends the speed and angle commands to the swerve module.
   * @param speed The desired speed. Domain: [-1, 1]
   * @param angle The desired angle. Domain: (-0.5, 0.5]
   */
  public void drive(double speed, double angle) {
    //if the opposite direction is closer to the current angle, flip the angle and the speed
    double[] optimizedState = optimize(
      speed,
      angle,
      turnRelativeEncoder.getPosition()
    );
    speed = optimizedState[0];
    angle = optimizedState[1];

    //sending the motor speed to the driving motor controller
    driveMotor.set(speed * SwerveConstants.speedMultiplier);

    //sending the motor angle to the turning motor controller
    turnPIDController.setReference(angle, CANSparkMax.ControlType.kPosition);

    //updatePIDFromShuffleboard();

    //updates the Shuffleboard tab
    updateShuffleboardTab(speed, angle);
  }

  public void reset() {
    turnRelativeEncoder.setPosition(
      turnCANcoder.getAbsolutePosition().getValueAsDouble() /
      SwerveConstants.kTurnConversionFactor
    );
  }

  public double[] optimize(double speed, double angle, double encoderAngle) {
    if (
      Math.abs(angle - encoderAngle) < 90 ||
      Math.abs(angle - encoderAngle) > 270
    ) {
      return new double[] { speed, angle };
    }

    return new double[] { -speed, ((angle + 1) % 1) - 0.5 };
  }

  public void createShuffleboardTab(String abbreviation) {
    sb_abbreviation = abbreviation;

    //learn how to use shuffleboard here:
    //https://docs.wpilib.org/en/stable/docs/software/dashboards/shuffleboard/layouts-with-code/index.html

    //creates the Shuffleboard tab
    sb_tab = Shuffleboard.getTab(sb_abbreviation);

    //creates the modifiable entries for the driving PID values
    sb_kDriveP = sb_tab.add("kDriveP", kDriveP).getEntry();
    sb_kDriveI = sb_tab.add("kDriveI", kDriveI).getEntry();
    sb_kDriveD = sb_tab.add("kDriveD", kDriveD).getEntry();
    sb_kDriveIZ = sb_tab.add("kDriveIZ", kDriveIZ).getEntry();
    sb_kDriveFF = sb_tab.add("kDriveFF", kDriveFF).getEntry();

    //creates the modifiable entries for the turning PID values
    sb_kTurnP = sb_tab.add("kTurnP", kTurnP).getEntry();
    sb_kTurnI = sb_tab.add("kTurnI", kTurnI).getEntry();
    sb_kTurnD = sb_tab.add("kTurnD", kTurnD).getEntry();
    sb_kTurnIZ = sb_tab.add("kTurnIZ", kTurnIZ).getEntry();
    sb_kTurnFF = sb_tab.add("kTurnFF", kTurnFF).getEntry();

    //for the calculated speed and angle of this swerve module on this iteration
    sb_speed = sb_tab.add("speed", 0).getEntry();
    sb_angle = sb_tab.add("angle", 0).getEntry();

    //for the reported angles from the encoders with the sparkMax
    sb_turnRelativeEncoderAngle = sb_tab.add("turnEncoderAngle", 0).getEntry();
    sb_turnCANcoderAngle = sb_tab.add("turnCANcoderAngle", 0).getEntry();
  }

  public void updateShuffleboardTab(double speed, double angle) {
    sb_speed.setDouble(speed);
    sb_angle.setDouble(angle);

    sb_turnRelativeEncoderAngle.setDouble(turnRelativeEncoder.getPosition());
    sb_turnCANcoderAngle.setDouble(
      turnCANcoder.getAbsolutePosition().getValueAsDouble()
    );
  }

  public void updatePIDFromShuffleboard() {
    //this method should only be used to iPID tuning; it should not be used during

    if (sb_kDriveP.getDouble(0) != kDriveP) {
      kDriveP = sb_kDriveP.getDouble(0);
      drivePIDController.setP(kDriveP);
    }
    if (sb_kDriveI.getDouble(0) != kDriveI) {
      kDriveI = sb_kDriveI.getDouble(0);
      drivePIDController.setI(kDriveI);
    }
    if (sb_kDriveD.getDouble(0) != kDriveD) {
      kDriveD = sb_kDriveD.getDouble(0);
      drivePIDController.setD(kDriveD);
    }
    if (sb_kDriveIZ.getDouble(0) != kDriveIZ) {
      kDriveIZ = sb_kDriveIZ.getDouble(0);
      drivePIDController.setIZone(kDriveIZ);
    }
    if (sb_kDriveFF.getDouble(0) != kDriveFF) {
      kDriveFF = sb_kDriveFF.getDouble(0);
      drivePIDController.setFF(kDriveFF);
    }

    if (sb_kTurnP.getDouble(0) != kTurnP) {
      kTurnP = sb_kTurnP.getDouble(0);
      turnPIDController.setP(kTurnP);
    }
    if (sb_kTurnI.getDouble(0) != kTurnI) {
      kTurnI = sb_kTurnI.getDouble(0);
      turnPIDController.setP(kTurnI);
    }
    if (sb_kTurnD.getDouble(0) != kTurnD) {
      kTurnD = sb_kTurnD.getDouble(0);
      turnPIDController.setD(kTurnD);
    }
    if (sb_kTurnIZ.getDouble(0) != kTurnIZ) {
      kTurnIZ = sb_kTurnIZ.getDouble(0);
      turnPIDController.setIZone(kTurnIZ);
    }
    if (sb_kTurnFF.getDouble(0) != kTurnFF) {
      kTurnFF = sb_kTurnFF.getDouble(0);
      turnPIDController.setFF(kTurnFF);
    }
  }
}
