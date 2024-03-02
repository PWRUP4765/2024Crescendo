// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotContainerConstants;
import frc.robot.commands.*;
import frc.robot.error.LimitException;
import frc.robot.subsystems.*;
import frc.robot.util.controller.LogitechController;
import frc.robot.util.controller.LogitechController.ButtonEnum;
import frc.robot.util.identity.Identity;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  // private final VisionSubsystem m_vision = new VisionSubsystem("limelight");
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private ClimbArmSubsystem climbArmSubsystem;

  final LogitechController m_driverController = new LogitechController(
    OperatorConstants.kDriverControllerPort
  );

  final LogitechController m_operatorController = new LogitechController(
    Constants.OperatorConstants.kOperatorControllerPort
  );

  final XboxController controller = new XboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    godbrigeroBindings();
    try {
      Identity identity = new Identity();
      this.getClass().getMethod(identity + "Bindings").invoke(this);
      //if (Objects.equals(identity.getIdentity(), "godbrigero")) return;
    } catch (Exception e) {
      e.printStackTrace();
    }

    // m_intake.setDefaultCommand(
    //   new RunCommand(
    //       () -> m_intake.setMotor(0), m_intake
    //   )
    // };

    if (RobotContainerConstants.kSwerveEnabled) {
      m_swerveSubsystem.setDefaultCommand(
        // 4765: Controller commands converted for various joysticks
        new RunCommand(
          () ->
            m_swerveSubsystem.joystickDrive(
              m_driverController.getRawAxis(0) * 1,
              m_driverController.getRawAxis(1) * -1,
              m_driverController.getRawAxis(2) * 1
            ),
          m_swerveSubsystem
        )
      );
    }

    if (RobotContainerConstants.kArmEnabled) {
      m_armSubsystem.setDefaultCommand(
        new RunCommand(() -> m_armSubsystem.updateFF(), m_armSubsystem)
      );
    }

    // try {
    //   climbArmSubsystem =
    //     new ClimbArmSubsystem(
    //       Constants.ClimbArmConstants.kClimbArmMotorPort,
    //       Constants.ClimbArmConstants.kClimbArmMotorIsBrushless
    //     );
    // } catch (NoChannelFoundException e) {
    //   e.printStackTrace();
    // }

    // climbArmSubsystem.setDefaultCommand(
    //   new RunCommand(
    //     () -> climbArmSubsystem.tick(false, controller.getBButton()),
    //     climbArmSubsystem
    //   )
    // );

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    /*new JoystickButton(
      m_driverController,
      LogitechController.ButtonEnum.A.value
    )
      .toggleOnTrue(
        new GoToAprilTag(
          m_driverController,
          m_swerveSubsystem,
          m_vision,
          0.0,
          0.0,
          0.0
        )
      );


     */

    if (RobotContainerConstants.kArmEnabled) {
      // When the x button on the LogitechController is pressed, we reset the position of the arm
      new JoystickButton(
        m_driverController,
        LogitechController.ButtonEnum.X.value
      )
        .onTrue(m_armSubsystem.setPositionCommand(0));
      // when the left trigger on the logitech controller is pressed, lets set the position of the arm to 0.125
      new JoystickButton(
        m_driverController,
        LogitechController.ButtonEnum.LEFTTRIGGER.value
      )
        .onTrue(m_armSubsystem.setPositionCommand(0.125));
      // When the y button is pressed on the logitech controller, lets set the position of the arm to 0.25 
      new JoystickButton(
        m_driverController,
        LogitechController.ButtonEnum.Y.value
      )
        .onTrue(m_armSubsystem.setPositionCommand(0.25));
    }
    // If the swerve drive is enabled, we should make it so that the start button resets the swerveSubsystem if it's getting buggy
    if (RobotContainerConstants.kSwerveEnabled) {
      new JoystickButton(
        m_driverController,
        LogitechController.ButtonEnum.STARTBUTTON.value
      )
        .onTrue(m_swerveSubsystem.runOnce(m_swerveSubsystem::reset));
    }
    // We should make it so that the back button of the logitech controller the intake runs once
    new JoystickButton(
      m_driverController,
      LogitechController.ButtonEnum.BACKBUTTON.value
    )
      .whileTrue(m_intake.runOnce(() -> m_intake.setMotor(-0.5)));

    // We should make it so that when the right trigger is pressed, the IntakeMotors start moving
    new JoystickButton(m_driverController, LogitechController.ButtonEnum.RIGHTTRIGGER.value)
      .toggleOnTrue(new IntakeCommand(m_intake, m_armSubsystem, m_swerveSubsystem));

    // We should make it so that when the right button is pressed, the IntakeMotors shoot out the note
    new JoystickButton(m_driverController, LogitechController.ButtonEnum.RIGHTBUTTON.value)
      .toggleOnTrue(new OutputCommand(m_intake));

    // Intake Button TBD
    //new Trigger(controller::getBButton)
    //  .toggleOnTrue(new IntakeCommand(m_intake, m_armSubsystem));
    // new Trigger(controller::getAButton)
    //  .toggleOnTrue(new ClimbArmCommand(climbArmSubsystem, 10, 0, "Climb Arm"));

    // m_chooser.addOption("Auton", new ExampleCommand(m_exampleSubsystem));
  }

  void godbrigeroBindings() {
    climbArmSubsystem =
      new ClimbArmSubsystem(Constants.ClimbArmConstants.kClimbArmMotorPort);

    new JoystickButton(m_operatorController, ButtonEnum.A.value)
      .whileTrue(
        new RunCommand(
          () -> {
            try {
              // Account for some error
              climbArmSubsystem.setSpeed(
                m_armSubsystem.getCurPosition() > 0.02 ? 0 : 50
              );
            } catch (LimitException e) {
              throw new RuntimeException(e);
            }
          },
          climbArmSubsystem
        )
      )
      .whileFalse(
        new RunCommand(
          () -> {
            try {
              climbArmSubsystem.setSpeed(
                m_operatorController.getRawButton(ButtonEnum.B.value) ? -50 : 0
              );
            } catch (LimitException e) {
              throw new RuntimeException(e);
            }
          },
          climbArmSubsystem
        )
      );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
