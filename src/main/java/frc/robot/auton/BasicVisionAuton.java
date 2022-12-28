package frc.robot.auton;

import java.util.Map;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.auton.trajectories.TestTrajectories;
import frc.robot.subsystems.DriveSubsystem;

public class BasicVisionAuton {

    // Determines possible selections which camera can make
    private enum TagSelector {
        ONE, // Default choice if all else fails
        TWO,
        THREE
    }

    public TestTrajectories trajs;
    private RamseteAutonSetup autonSetup;
    
    private DriveSubsystem drive;
    private PhotonCamera camera;
    
    public BasicVisionAuton(DriveSubsystem drive, PhotonCamera camera, RamseteAutonSetup autonSetup, TrajectoryConfig config) {

        this.autonSetup = autonSetup;
        this.drive = drive;
        this.camera = camera;

        trajs = new TestTrajectories(config);

    }

    // Gets detected tag from camera, or ONE as default
    private TagSelector selectTag() {
        var result = camera.getLatestResult();

        if (result.hasTargets()) {
          var target = result.getBestTarget();
        
          switch (target.getFiducialId()) {
            case 2:
              return TagSelector.TWO;
            case 3:
              return TagSelector.THREE;
            default:
              return TagSelector.ONE;
          }
        } else {
          return TagSelector.ONE;
        }
    }

    public Pose2d getStartPose() {
        return trajs.startPose;
    }

    // Creates command which runs trajectories based upon what tag was detected
    public Command genSelectCommand() {
        return new SelectCommand(
            Map.ofEntries(
                Map.entry(TagSelector.ONE, autonSetup.getRamseteCommand(trajs.traj1)),
                Map.entry(TagSelector.TWO, autonSetup.getRamseteCommand(trajs.traj2)),
                Map.entry(TagSelector.THREE, autonSetup.getRamseteCommand(trajs.traj3))
            ),
            this::selectTag
        );
    }

}
