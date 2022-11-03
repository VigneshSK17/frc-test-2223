package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
    
    // Drivetrain motors
    private final CANSparkMax leftMain  = new CANSparkMax(DriveConstants.LEFT_MOTOR_1, MotorType.kBrushless);
    private final CANSparkMax leftFollower  = new CANSparkMax(DriveConstants.LEFT_MOTOR_2, MotorType.kBrushless);
    private final CANSparkMax rightMain  = new CANSparkMax(DriveConstants.RIGHT_MOTOR_1, MotorType.kBrushless);
    private final CANSparkMax rightFollower  = new CANSparkMax(DriveConstants.RIGHT_MOTOR_2, MotorType.kBrushless);

    private final DifferentialDrive drive;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    public DriveSubsystem() {

        // Have to invert right side
        rightMain.setInverted(true);

        // Makes left and right side move the same
        leftFollower.follow(leftMain);
        rightFollower.follow(rightMain);

        // Creates drive
        drive = new DifferentialDrive(leftMain, rightMain);

    }

    // Pass in values, mainly from controller, to control x axis and rotation speed
    public void drive(double xSpeed, double rotation) {
        drive.arcadeDrive(xSpeed, rotation, true);
    }

    // Sets max output for the drivetrain, which is in drive constants
    public void setToMaxOutput() {
        drive.setMaxOutput(DriveConstants.MAX_OUTPUT);
    }

    // Sets the slow mode output for the drivetrain, which is in drive constants
    public void setToSlowOutput() {
        drive.setMaxOutput(DriveConstants.MIN_OUTPUT);
    }

}
