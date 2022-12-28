package frc.robot.auton.trajectories;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public class TestTrajectories {
    
    TrajectoryConfig config;
    public Pose2d startPose = new Pose2d(0, 0, new Rotation2d(0));

    public TestTrajectories(TrajectoryConfig config) {
        this.config = config;
    }

    public Trajectory traj1 =
      TrajectoryGenerator.generateTrajectory(
        startPose,
        new ArrayList<>(),
        new Pose2d(3, 0, new Rotation2d(0)),
        config
      );

    public Trajectory traj2 =
      TrajectoryGenerator.generateTrajectory(
        startPose,
        new ArrayList<>(),
        new Pose2d(-3, 0, new Rotation2d(0)),
        config
      );

    public Trajectory traj3 =
      TrajectoryGenerator.generateTrajectory(
        startPose,
        List.of(
            new Translation2d(3, 0)
        ),
        new Pose2d(3, 3, new Rotation2d()),
        config
      );
}
