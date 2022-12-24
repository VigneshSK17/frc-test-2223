package frc.robot.auton;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * A wrapper class which only asks for DriveSubsystem at init
 * and trajectory when trying to use a RamseteCommand
 */
public class RamseteAutonSetup {
    
    private DriveSubsystem drive;

    public RamseteAutonSetup(DriveSubsystem drive) {
        this.drive = drive;
    }

    public RamseteCommand getRamseteCommand(Trajectory traj) {
        return new RamseteCommand(
            traj,
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

    private DifferentialDriveVoltageConstraint getVoltageConstraint() {
        return new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.S_VOLTS,
                DriveConstants.V_VOLT_SECONDS_PER_METER,
                DriveConstants.A_VOLT_SECONDS_SQUARED_PER_METER
            ),
            DriveConstants.DRIVE_KINEMATICS,
            10
        );
    }

    public TrajectoryConfig getTrajectoryConfig() {
        return new TrajectoryConfig(
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED
        )
        .setKinematics(DriveConstants.DRIVE_KINEMATICS)
        .addConstraint(this.getVoltageConstraint());
    }

}
