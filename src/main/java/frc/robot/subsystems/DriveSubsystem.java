package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
    
    // Drivetrain motors
    private final CANSparkMax leftMain  = new CANSparkMax(DriveConstants.LEFT_MOTOR_1, MotorType.kBrushless);
    private final CANSparkMax leftFollower  = new CANSparkMax(DriveConstants.LEFT_MOTOR_2, MotorType.kBrushless);
    private final CANSparkMax rightMain  = new CANSparkMax(DriveConstants.RIGHT_MOTOR_1, MotorType.kBrushless);
    private final CANSparkMax rightFollower  = new CANSparkMax(DriveConstants.RIGHT_MOTOR_2, MotorType.kBrushless);

    private final DifferentialDrive drive;

    // Drivetrain encoders stuff
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final Gyro gyro = new ADXRS450_Gyro();

    private final DifferentialDriveOdometry odometry;
    private final DifferentialDriveKinematics kinematics;

    public DriveSubsystem() {

        // Have to invert right side
        rightMain.setInverted(true);

        // Makes left and right side move the same
        leftFollower.follow(leftMain);
        rightFollower.follow(rightMain);

        /* Odometry Setup */
        leftEncoder = leftMain.getEncoder();
        rightEncoder = rightMain.getEncoder();

        leftEncoder.setPositionConversionFactor(DriveConstants.ENCODER_DISTANCE_PER_PULSE);
        rightEncoder.setPositionConversionFactor(DriveConstants.ENCODER_DISTANCE_PER_PULSE);

        resetEncoders();
        odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
        kinematics = new DifferentialDriveKinematics(DriveConstants.TRACKWIDTH);

        // Creates drive
        drive = new DifferentialDrive(leftMain, rightMain);

    }

    // Loops throughout the lifetime of the subsystem
    @Override
    public void periodic() {
        odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
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

    /* Autonomous stuff */

    // Resets encoders
    public void resetEncoders() {
        leftEncoder.setPosition(0.0);
        rightEncoder.setPosition(0.0);
    }

    // Resets gyroscope, zeroes heading of robot
    public void resetGyro() {
        gyro.reset();
    }

    /**
     *  Returns heading of robot 
     * 
     * @return The robot's heading in degrees (-180 to 180)
     */
    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    /**
     * Returns turn rate of robot
     * 
     * @return The turn rate of the robot in degrees per second
     */
    public double getTurnRate() {
        return -gyro.getRate();
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
      return odometry.getPoseMeters();
    }

     /**
    * Returns the current wheel speeds of the robot.
    *
    * @return The current wheel speeds.
    */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
      return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
      resetEncoders();
      odometry.resetPosition(pose, gyro.getRotation2d());
    }


    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts the commanded left output
     * @param rightVolts the commanded right output
     */
    public void driveVolts(double leftVolts, double rightVolts) {
      leftMain.setVoltage(leftVolts);
      rightMain.setVoltage(rightVolts);
      drive.feed();
    }

    /**
     * Gets the average distance of the two encoders
     * 
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
    }

    public RelativeEncoder getLeftEncoder() {
        return leftEncoder;
    }

    public RelativeEncoder getRightEncoder() {
        return rightEncoder;
    }

}
