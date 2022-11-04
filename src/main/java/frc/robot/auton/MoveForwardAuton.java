package frc.robot.auton;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;


public class MoveForwardAuton extends SequentialCommandGroup {
    
    public MoveForwardAuton(DriveSubsystem driveSubsystem) {

        addCommands(
            new RunCommand(() -> driveSubsystem.drive(-0.8, 0), driveSubsystem)
                .withTimeout(1.0) 
        );

    }

}