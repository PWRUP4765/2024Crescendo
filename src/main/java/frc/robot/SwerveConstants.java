package frc.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;

public class SwerveConstants {

  //the driving motor ports
  public static final int kFrontLeftDriveMotorPort = 10;
  public static final int kFrontRightDriveMotorPort = 21;
  public static final int kRearLeftDriveMotorPort = 12;
  public static final int kRearRightDriveMotorPort = 13;

  //whether the driving encoders are flipped
  public static final boolean kFrontLeftDriveMotorReversed = true;
  public static final boolean kRearLeftDriveMotorReversed = true;
  public static final boolean kFrontRightDriveMotorReversed = true;
  public static final boolean kRearRightDriveMotorReversed = true;

  //the turning motor ports
  public static final int kFrontLeftTurningMotorPort = 22;
  public static final int kFrontRightTurningMotorPort = 20;
  public static final int kRearLeftTurningMotorPort = 25;
  public static final int kRearRightTurningMotorPort = 23;

  //whether the turning enoders are flipped
  public static final boolean kFrontLeftTurningMotorReversed = false;
  public static final boolean kFrontRightTurningMotorReversed = false;
  public static final boolean kRearLeftTurningMotorReversed = false;
  public static final boolean kRearRightTurningMotorReversed = false;

  //the CANCoder turning encoder ports
  public static final int kFrontLeftCANcoderPort = 3;
  public static final int kFrontRightCANcoderPort = 4;
  public static final int kRearLeftCANcoderPort = 2;
  public static final int kRearRightCANcoderPort = 1;

  //whether the turning CANCoders are flipped
  public static final SensorDirectionValue kFrontLeftCANcoderDirection =
    SensorDirectionValue.Clockwise_Positive;
  public static final SensorDirectionValue kFrontRightCANcoderDirection =
    SensorDirectionValue.Clockwise_Positive;
  public static final SensorDirectionValue kRearLeftCANcoderDirection =
    SensorDirectionValue.Clockwise_Positive;
  public static final SensorDirectionValue kRearRightCANcoderDirection =
    SensorDirectionValue.Clockwise_Positive;

  //magnetic offset for the CANCoders
  //you can find these by connecting to the RoboRIO by USB on the drive station, opening the Phoenix Tuner app, and taking snapshots of
  //the rotational values of the CANCoders while in they are in the forward state
  public static final double kFrontLeftCANcoderMagnetOffset = -0.403; //-0.403 rotations
  public static final double kFrontRightCANcoderMagnetOffset = -0.250; //-0.250 rotations
  public static final double kRearLeftCANcoderMagnetOffset = -0.422; //-0.422 rotations
  public static final double kRearRightCANcoderMagnetOffset = 0.064; //0.064 rotations

  //stats used by SwerveSubsystem for math
  public static final double kDriveBaseWidth = 0.47625;
  public static final double kDriveBaseLength = 0.47625;

  //stats used by SwerveSubsystem for deadbanding
  public static final double kXSpeedDeadband = 0.05;
  public static final double kXSpeedMinValue = 0;
  public static final double kYSpeedDeadband = 0.05;
  public static final double kYSpeedMinValue = 0;
  public static final double kRotDeadband = 0.05;
  public static final double kRotMinValue = 0;

  //disable this if you want to try non-fieldRelative driving
  public static final boolean kFieldRelative = true;

  //PID values for the driving
  public static final double kDriveP = 0.01;
  public static final double kDriveI = 0;
  public static final double kDriveD = 0;
  public static final double kDriveIZ = 0;
  public static final double kDriveFF = 0;
  public static final double kDriveMinOutput = -1;
  public static final double kDriveMaxOutput = 1;
  //multiplies the output speed of all of the drive motors, ALWAYS (0, 1).
  public static final double speedMultiplier = 0.25;
  //the way that the drive motor controller sends power when not receiving a signal from the code. (kBrake or kCoast)
  public static final CANSparkMax.IdleMode kDriveIdleMode =
    CANSparkMax.IdleMode.kBrake;
  public static final double kDriveMaxRPM = 5700;

  //PID values for the turning
  public static final double kTurnP = 2;
  public static final double kTurnI = 0.0015;
  public static final double kTurnD = 0.12;
  public static final double kTurnIZ = 0;
  public static final double kTurnFF = 0;
  public static final double kTurnMinOutput = -1;
  public static final double kTurnMaxOutput = 1;
  //because the turn gearing ratio is not 1:1, we need to spin the motor many times to equal one spin of the module
  //this constant is used for the position conversion factor. (every 150 turns of motors is 7 rotations of the module)
  public static final double kTurnConversionFactor = 7.0 / 150.0;
}
