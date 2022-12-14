// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Robot subsystems
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();

  // Controller
  private final XboxController controller = new XboxController(IOConstants.DRIVER_CONTROLLER_PORT_1);

  // Creates custom Trigger if the left trigger is pressed more than 70%
  private Trigger leftTrigger = new Trigger(() -> controller.getLeftTriggerAxis() > 0.3);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Sets default command for drive subsystem
    driveSubsystem.setDefaultCommand(new RunCommand(
      () -> driveSubsystem.drive(-controller.getLeftY(), controller.getRightX()),
      driveSubsystem
    ));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Sets drive subsystem speed if 
    leftTrigger
      .whenActive(
        () -> driveSubsystem.setToMaxOutput(),
        driveSubsystem
      )
      .whenInactive(
        () -> driveSubsystem.setToSlowOutput(),
        driveSubsystem
      );

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new ExampleCommand(exampleSubsystem);
  }
}
