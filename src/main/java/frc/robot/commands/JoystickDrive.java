package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.hardware.HardwareComponents;
import frc.robot.hardware.NavX;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.MathFunc;
import frc.robot.util.controller.FlightModule;
import frc.robot.util.controller.FlightStick;

public class JoystickDrive extends Command {
    
    private final NavX m_gyro = HardwareComponents.gyro;
    private final SwerveSubsystem m_swerveSubsystem;

    private final FlightModule m_controllers;

    private PIDController m_directionPIDController = new PIDController(
        SwerveConstants.kDirectionP,
        SwerveConstants.kDirectionI,
        SwerveConstants.kDirectionD
    );
    
    private double desiredDirection;
    private boolean pidDirection = false;
    private int countUntilPid = 0;

    public JoystickDrive(SwerveSubsystem swerveSubsystem, FlightModule controllers) {
        m_swerveSubsystem = swerveSubsystem;
        m_controllers = controllers;

        addRequirements(m_swerveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double x = MathFunc.deadband(
            m_controllers.rightFlightStick.getRawAxis(FlightStick.AxisEnum.JOYSTICKX.value),
            SwerveConstants.kXSpeedDeadband,
            SwerveConstants.kXSpeedMinValue
        );
        double y = MathFunc.deadband(
            m_controllers.rightFlightStick.getRawAxis(FlightStick.AxisEnum.JOYSTICKY.value),
            SwerveConstants.kYSpeedDeadband,
            SwerveConstants.kYSpeedMinValue
        );
        double r = MathFunc.deadband(
            m_controllers.leftFlightStick.getRawAxis(FlightStick.AxisEnum.JOYSTICKROTATION.value),
            SwerveConstants.kRotDeadband,
            SwerveConstants.kRotMinValue
        );

    //adjusting for field relativity if necessary
    if (SwerveConstants.kFieldRelative) {
      double gyroAngle = m_gyro.getAngle(); //this gets the angle and puts it from -1/2 to 1/2
      double nonFieldRelativeAngle = Math.atan2(x, y) / (2 * Math.PI); //again, the return value is from -1/2 to 1/2
      double fieldRelativeAngle = nonFieldRelativeAngle - gyroAngle;

      double magnitude = Math.sqrt((x * x) + (y * y));

      x = Math.sin(fieldRelativeAngle * 2 * Math.PI) * magnitude;
      y = Math.cos(fieldRelativeAngle * 2 * Math.PI) * magnitude;
    }
    if (Math.abs(r) > 0) {
      pidDirection = false;
      desiredDirection = m_gyro.getAngle();
      countUntilPid = 0;
    } else if (countUntilPid < 25) {
      desiredDirection = m_gyro.getAngle();
      countUntilPid++;
    } else {
      pidDirection = true;
    }

    if (SwerveConstants.kPIDDirection && pidDirection) {
      desiredDirection = MathFunc.putWithinHalfToHalf(desiredDirection + (r * SwerveConstants.kDirectionMultiplier));
      r = m_directionPIDController.calculate(m_gyro.getAngle(), desiredDirection);
    }

    m_swerveSubsystem.drive(x, y, r);
    }

    
}
