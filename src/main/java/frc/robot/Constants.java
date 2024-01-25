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
    public static final double kArmP = 0;
    public static final double kArmI = 0;
    public static final double kArmD = 0;
    public static final double kArmIZ = 0;
    public static final double kArmFF = 0;
    public static final double kArmMinOutput = -1;
    public static final double kArmMaxOutput = 1;

    public static final double kArmEncoderOffset = 0;
    public static final double kArmEncoderConversionFactor = 1;

    public static final double kArmMinPosition = 0;
    public static final double kArmMaxPosition = 1;
  }

  public static class ClimbArmConstants {

    public static final int kClimbArmMotorPort = 0; // TEMP
    public static final boolean kClimbArmMotorIsBrushless = true; // TEMP
  }

  public static class IntakeConstants {

    // Motor Speeds
    public static final float kIntakeSpeed = 0.2F; // TEMP
    public static final float kShooterSpeed = 0.8F; // TEMP

    // Intake Paramters
    public static final int kIntakePort = 0; // TEMP
    public static final int kShooterPort1 = 1; // TEMP
    public static final int kShooterPort2 = 2; // TEMP
    public static final boolean kIsBrushless = true; // TEMP

    public static final boolean kIsIntakeReversed = false; // TEMP
    public static final boolean kIsShooter1Reversed = false; // TEMP
    public static final boolean kIsShooter2Reversed = true; // TEMP
  }
}
