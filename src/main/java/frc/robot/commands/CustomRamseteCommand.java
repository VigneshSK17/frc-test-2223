package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * A wrapper class which only asks for DriveSubsystem at init
 * and trajectory when trying to use a RamseteCommand
 */
public class CustomRamseteCommand {
    
    private DriveSubsystem drive;

    public CustomRamseteCommand(DriveSubsystem drive) {
        this.drive = drive;
    }

    public RamseteCommand getRamseteCommand(Trajectory trajectory) {
        return new RamseteCommand(
            trajectory,
            drive::getPose,
            new RamseteController(AutoConstants.RAMSETE_B, AutoConstants.RAMSETE_ZETA),
            new SimpleMotorFeedforward(
                DriveConstants.S_VOLTS,
                DriveConstants.V_VOLT_SECONDS_PER_METER,
                DriveConstants.A_VOLT_SECONDS_SQUARED_PER_METER
            ),
            DriveConstants.DRIVE_KINEMATICS,
            drive::getWheelSpeeds,
            new PIDController(DriveConstants.P_DRIVE_VEL, 0, 0),
            new PIDController(DriveConstants.P_DRIVE_VEL, 0, 0),
            drive::driveVolts,
            drive
        );
    }

}
