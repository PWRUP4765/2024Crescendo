// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  }

  public static class ArmConstants {

    public static final int kArmMotorPort = 0; // TEMP
    public static final boolean kArmMotorReversed = false; // TEMP

    // Arm PID constants
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kIZ = 0;
    public static final double kFF = 0;
    public static final double kMinOutput = -1;
    public static final double kMaxOutput = 1;

    public static final double kEncoderOffset = 0;
    public static final double kEncoderConversionFactor = 5.0 / 8.0;

    public static final double kMinPosition = 0;
    public static final double kMaxPosition = 0.5;

    public static final double kFFCoefficient = 0;

    public static final double kArmFlatPosition = 0;
    public static final double kArmDefaultPosition = 0; // TEMP
    public static final double kArmScoringPosition = 0; // TEMP
  }

  public static class ClimbArmConstants {

    public static final int kClimbArmMotorPort = 0; // TEMP
    public static final boolean kClimbArmMotorIsBrushless = true; // TEMP

    public static final double kClimbArmLengthMeters = 1; // TMP
    public static final double kClimbArmMinLengthMeters = 1; // TMP
    public static final double kClimbGearDiameterMeters = 0.2; // TMP

    public static final double kProportionalGain = 0; // FIXME: will need to be changed!
    public static final double kIntegralGain = 0; // FIXME: will need to be changed!
    public static final double kDerivativeGain = 0; // FIXME: will need to be changed!

    public static final double kIZone = 0; // FIXME: will need to be changed!
    public static final double kFeedForward = 0; // FIXME: will need to be changed!
  }

  public static class IntakeConstants {
    public static final String kCanbusAddress = ""; //TEMP

    // Motor Speeds
    public static final float kIntakeSpeed = 0.2F; // TEMP
    public static final float kShooterSpeed = 0.8F; // TEMP

    // Intake Paramters
    public static final int kIntakePort = 0; // TEMP
    public static final int kShooterPort1 = 1; // TEMP
    public static final int kShooterPort2 = 2; // TEMP
    public static final boolean kIsBrushless = false; // TEMP

    public static final boolean kIsIntakeReversed = false; // TEMP
    public static final boolean kIsShooter1Reversed = false; // TEMP
    public static final boolean kIsShooter2Reversed = true; // TEMP
  }
}
