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
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.GoToAprilTag;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.RobotContainerConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ClimbArmCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.error.LimitException;
import frc.robot.error.NoChannelFoundException;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimbArmSubsystem;
import frc.robot.util.controller.LogitechController;
import frc.robot.util.controller.LogitechController.ButtonEnum;


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
  private final VisionSubsystem m_vision = new VisionSubsystem("limelight");
  private final SwerveSubsystem m_swerveSubsystem;
  private ArmSubsystem m_armSubsystem;
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final double intakeSpeed = 0.5;
  private ClimbArmSubsystem climbArmSubsystem;

  final Joystick m_driverController = new Joystick(
    OperatorConstants.kDriverControllerPort
  );

  final XboxController controller = new XboxController(0);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    m_intake.setDefaultCommand(
      new RunCommand(
          () -> m_intake.setMotor(), m_intake
      )
    );
    
    if (RobotContainerConstants.kSwerveEnabled) {
      m_swerveSubsystem = new SwerveSubsystem();
      m_swerveSubsystem.setDefaultCommand(
        // 4765: Controller commands converted for various joysticks
        new RunCommand(
          () ->
            m_swerveSubsystem.joystickDrive(
              m_driverController.getRawAxis(0) * 1,
              m_driverController.getRawAxis(1) * -1,
              m_driverController.getRawAxis(2) * 1
            ), m_swerveSubsystem
        )
      );
    }
    
    if (RobotContainerConstants.kArmEnabled) {
      m_armSubsystem = new ArmSubsystem();
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
    new JoystickButton(m_driverController, 2)
      .toggleOnTrue(new GoToAprilTag(m_swerveSubsystem, m_vision, 0.0, 0.0, 0.0));
    

    if (RobotContainerConstants.kArmEnabled) {
      new JoystickButton(m_driverController, LogitechController.ButtonEnum.X.value)
        .onTrue(m_armSubsystem.setPositionCommand(0));
      new JoystickButton(m_driverController, LogitechController.ButtonEnum.Y.value)
        .onTrue(m_armSubsystem.setPositionCommand(0.25));
    }

    if (RobotContainerConstants.kSwerveEnabled) {
      new JoystickButton(m_driverController, LogitechController.ButtonEnum.STARTBUTTON.value)
        .onTrue(m_swerveSubsystem.runOnce(() -> m_swerveSubsystem.reset()));
    }
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    new Trigger(m_exampleSubsystem::exampleCondition)
      .onTrue(new ExampleCommand(m_exampleSubsystem));

    // new JoystickButton(m_operatorController, LogitechController.ButtonEnum.B.value)
    //   .toggleOnTrue(new IntakeCommand(m_intake, m_armSubsystem));

    // Intake Button TBD
    //new Trigger(controller::getBButton)
    //  .toggleOnTrue(new IntakeCommand(m_intake, m_armSubsystem));
    // FIXME: test @this.
    // new Trigger(controller::getAButton)
    //  .toggleOnTrue(new ClimbArmCommand(climbArmSubsystem, 10, 0, "Climb Arm"));

    m_chooser.addOption("Auton", new ExampleCommand(m_exampleSubsystem));
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
