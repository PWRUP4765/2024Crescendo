package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem {

  private CANSparkMax inputMotor;
  private CANSparkMax shooterMotor1;
  private CANSparkMax shooterMotor2;

  public void IntakeSubsytem(
    int inputMotorChannel,
    int sensorChannel,
    int shooterMotorChannel,
    int shooterMotorChannel2
  ) {
    inputMotor = new CANSparkMax(inputMotorChannel, MotorType.kBrushless);
    shooterMotor1 = new CANSparkMax(shooterMotorChannel, MotorType.kBrushless);
    shooterMotor2 = new CANSparkMax(shooterMotorChannel2, MotorType.kBrushless);
  }
}
