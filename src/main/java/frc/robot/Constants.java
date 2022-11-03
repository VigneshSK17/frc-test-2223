// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /**
     * Contains the motor ports for the drivetrain
     */
    public static final class DriveConstants {

        // Left drive
        public static final int LEFT_MOTOR_1 = 5;
        public static final int LEFT_MOTOR_2 = 6;

        // Right drive
        public static final int RIGHT_MOTOR_1 = 7;
        public static final int RIGHT_MOTOR_2 = 8;

        // Drive motor speeds
        public static final double MAX_OUTPUT = 0.8;
        public static final double MIN_OUTPUT = 0.2;

        // Kinematics Constants
        public static final double TRACKWIDTH = Units.inchesToMeters(20);
        public static final DifferentialDriveKinematics DRIVE_KINEMATICS = 
            new DifferentialDriveKinematics(DriveConstants.TRACKWIDTH);

        public static final double WHEEL_DIAMETER = Units.inchesToMeters(6);
        public static final int ENCODER_COUNTS_PER_REVOLUTION = 8096;
        public static final double ENCODER_DISTANCE_PER_PULSE = 
            (WHEEL_DIAMETER * Math.PI) / (double) ENCODER_COUNTS_PER_REVOLUTION;

        // Voltage
        // TODO: Find values using Robot Characterization Toolsuite
        public static final double S_VOLTS = 0.22;
        public static final double V_VOLT_SECONDS_PER_METER = 1.98;
        public static final double A_VOLT_SECONDS_SQUARED_PER_METER = 0.2;

        // Drive velocity
        // TODO: Find values using Robot Characterization Toolsuite
        public static final double P_DRIVE_VEL = 8.5;
    }

    /**
     * Contains ports for various input and output devices
     */
    public static final class IOConstants {

        // Xbox Controller
        public static final int DRIVER_CONTROLLER_PORT_1 = 0;

    }

    public static final class AutoConstants {
        
        // Ramsete follower values
        // TODO: Tune these
        public static final double RAMSETE_B = 2;
        public static final double RAMSETE_ZETA = 0.7;

    }

}
