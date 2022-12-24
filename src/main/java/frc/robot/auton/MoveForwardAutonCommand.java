package frc.robot.auton;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MoveForwardAutonCommand extends SequentialCommandGroup {

    private TrajectoryConfig config;

    public MoveForwardAutonCommand(RamseteAutonSetup autonSetup, TrajectoryConfig config) {
        this.config = config;

        addCommands(
            autonSetup.getRamseteCommand(traj0)
        ); 
    }

    // Moves forward by 3 meters
    // TODO: Make every other trajectory private
    public Trajectory traj0 = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            new ArrayList<>(),
            new Pose2d(3, 0, new Rotation2d(0)),
            config);
    

}
