package frc.robot.commands;

import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auton.RamseteAutonSetup;
import frc.robot.subsystems.DriveSubsystem;

public class TurnBotVisionCommand extends SequentialCommandGroup {

    final double CAMERA_HEIGHT = Units.inchesToMeters(24); // On bot
    final double TARGET_HEIGHT = Units.feetToMeters(5);

    // Angle between horiz and camera (radians)
    final double CAMERA_PITCH = Units.degreesToRadians(0);

    public TurnBotVisionCommand(DriveSubsystem drive, RamseteAutonSetup autonSetup, TrajectoryConfig config) {

        PhotonCamera camera = new PhotonCamera("daZed");

        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {

            // Degrees of turn needed.
            double yaw = result.getBestTarget().getYaw();

            // Create trajectory to turn
            Trajectory traj = 
                TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0, 0, new Rotation2d(0)),
                    new ArrayList<>(),
                    new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(yaw))),
                    config);

            drive.resetOdometry(traj.getInitialPose());

            addCommands(
                autonSetup.getRamseteCommand(traj),
                new InstantCommand(() -> drive.driveVolts(0, 0))
            );

        } else {
            addCommands();
        }

    }

}
