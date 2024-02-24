// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {

    public static final int kDriverControllerPort = 0;

    public static final int kDriveTeamConstants2 = 1;
  }

  public static class RobotContainerConstants {

    public static final boolean kSwerveEnabled = true;
    public static final boolean kArmEnabled = false;
  }

  public static class ArmConstants {

    public static final int kArmMotorPort = 32;
    public static final boolean kArmMotorReversed = false;

    // Arm PID constants
    public static final double kP = 2.5; // 2
    public static final double kI = 0; // 0.002
    public static final double kD = 4; // 2
    public static final double kIZ = 0;
    public static final double kFF = 0;
    public static final double kFFCoefficient = 2.4;

    public static final boolean kEncoderReversed = false;
    public static final double kEncoderOffset = 0.258;
    public static final double kEncoderConversionFactor = 1.0;

    public static final double kMinPosition = 0;
    public static final double kMaxPosition = 0.25;

    public static final double kArmFlatPosition = 0;
    public static final double kArmDefaultPosition = 0; // TEMP
    public static final double kArmScoringPosition = 0.25; // TEMP
  }

  public static class ClimbArmConstants {

    public static final int kClimbArmMotorPort = 32;
    public static final boolean kClimbArmMotorIsBrushless = false;

    public static final double kClimbArmLengthMeters = 1; // FIXME: will need to be changed!
    public static final double kClimbArmMinLengthMeters = 1; // FIXME: will need to be changed!
    public static final double kClimbGearDiameterMeters = 0.2; // FIXME: will need to be changed!

    public static final double kProportionalGain = 0; // FIXME: will need to be changed!
    public static final double kIntegralGain = 0; // FIXME: will need to be changed!
    public static final double kDerivativeGain = 0; // FIXME: will need to be changed!

    public static final double kIZone = 0; // FIXME: will need to be changed!
    public static final double kFeedForward = 0; // FIXME: will need to be changed!
  }

  public static class IntakeConstants {

    public static final String kCanbusAddress = ""; //TEMP
    public static final int kCurrentLimit = 40;

    // Motor Speeds
    public static final double kIntakeSpeed = 0.5; // TEMP
    public static final double kShooterSpeed = 0.8; // TEMP

    // Intake Paramters
    public static final int kIntakePort = 30; // TEMP
    public static final int kShooterPort1 = 1; // TEMP
    public static final int kShooterPort2 = 2; // TEMP
    public static final boolean kIsBrushless = false; // TEMP

    public static final boolean kIsIntakeReversed = false; // TEMP
    public static final boolean kIsShooter1Reversed = false; // TEMP
    public static final boolean kIsShooter2Reversed = true; // TEMP
  }

  public class SwerveConstants {

    //the driving motor ports
    public static final int kFrontLeftDriveMotorPort = 13;
    public static final int kFrontRightDriveMotorPort = 25;
    public static final int kRearLeftDriveMotorPort = 23;
    public static final int kRearRightDriveMotorPort = 22;

    //whether the driving encoders are flipped
    public static final boolean kFrontLeftDriveMotorReversed = true;
    public static final boolean kRearLeftDriveMotorReversed = true;
    public static final boolean kFrontRightDriveMotorReversed = true;
    public static final boolean kRearRightDriveMotorReversed = true;

    //the turning motor ports
    public static final int kFrontLeftTurningMotorPort = 20;
    public static final int kFrontRightTurningMotorPort = 10;
    public static final int kRearLeftTurningMotorPort = 21;
    public static final int kRearRightTurningMotorPort = 12;

    //whether the turning enoders are flipped
    public static final boolean kFrontLeftTurningMotorReversed = false;
    public static final boolean kFrontRightTurningMotorReversed = false;
    public static final boolean kRearLeftTurningMotorReversed = false;
    public static final boolean kRearRightTurningMotorReversed = false;

    //the CANCoder turning encoder ports - updated 2/12/24
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
    public static final double kFrontLeftCANcoderMagnetOffset = 0.097;
    public static final double kFrontRightCANcoderMagnetOffset = 0.252;
    public static final double kRearLeftCANcoderMagnetOffset = 0.079;
    public static final double kRearRightCANcoderMagnetOffset = -0.441;

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
    public static final double kSpeedMultiplier = 0.25;
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

  public static class VisionConstants {

    // All units are in meters

    // Constants such as camera and target height stored. Change per robot and goal!
    public static final double kCAMERA_HEIGHT_METERS = Units.inchesToMeters(24);

    public static final double kTARGET_HEIGHT_METERS = Units.feetToMeters(5);
    // Angle between horizontal and the camera.
    public static final double kCAMERA_PITCH_RADIANS = Units.degreesToRadians(
      0
    );

    // How far from the target we want to be

    public static final double kAmpXGoal = 1;
    public static final double kAmpYGoal = 1;
    public static final double kAmpRotGoal = 0;
    public static final double kXP = 0.2;
    public static final double kXD = 0.0;
    public static final double kYP = 0.2;
    public static final double kYD = 0.0;
    public static final double kRP = 0.2;
    public static final double kRD = 0.0;
    // public static final double kLINEAR_P = 0.2;

    // public static final double kLINEAR_D = 0.0;

    // public static final double kANGULAR_P = 0.2;

    // public static final double kANGULAR_D = 0.0;
  }
}
