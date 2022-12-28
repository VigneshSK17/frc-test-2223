// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.IOConstants;
import frc.robot.auton.BasicVisionAuton;
import frc.robot.auton.RamseteAutonSetup;
import frc.robot.commands.TurnBotVisionCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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

  // Enables the use of Ramsete paths
  private final RamseteAutonSetup autonSetup = new RamseteAutonSetup(driveSubsystem);
  private final TrajectoryConfig trajectoryConfig = autonSetup.getTrajectoryConfig();

  // Photoncamera setup
  private final PhotonCamera camera = new PhotonCamera("daZed");

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
        () -> driveSubsystem.setToSlowOutput(),
        driveSubsystem
      )
      .whenInactive(
        () -> driveSubsystem.setToMaxOutput(),
        driveSubsystem
      );

      // Turns bot to what camera wants to detect when pressed, if it can't detect does nothing
      new JoystickButton(controller, Button.kA.value).whenPressed(
        new TurnBotVisionCommand(driveSubsystem, camera, autonSetup, trajectoryConfig)
      );

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // TODO: Uncomment when there is no teleop use case
    // final RamseteAutonSetup autonSetup = new RamseteAutonSetup(driveSubsystem);
    // final TrajectoryConfig trajectoryConfig = autonSetup.getTrajectoryConfig();

    //region No Vision Auton
    // final MoveForwardAutonCommand auton = new MoveForwardAutonCommand(autonSetup, trajectoryConfig);

    // // Resets odometry to beginning of first path
    // driveSubsystem.resetOdometry(auton.traj0.getInitialPose());

    // // Runs ramsete commands, ends by setting drive volts to 0 to stop bot
    // return auton.andThen(() -> driveSubsystem.driveVolts(0, 0));
    //endregion

    //region Vision Auton (w/ default methods)

    BasicVisionAuton auton = new BasicVisionAuton(driveSubsystem, camera, autonSetup, trajectoryConfig);

    // Reset odom to beginning of all paths
    driveSubsystem.resetOdometry(auton.getStartPose());

    // Runs ramsete command based on what camera detects
    return auton.genSelectCommand().andThen(() -> driveSubsystem.driveVolts(0, 0));

    //endregion
  }
}
