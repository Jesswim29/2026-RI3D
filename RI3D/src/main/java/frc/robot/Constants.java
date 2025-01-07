// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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
    public static final int pigeonID = 5;
    public static final int kButtonController = 0;
    public static final int kDriveController = 1;
    public static final double kDeadzone = 0.05;

    public static final class DrivetrainConstants {
        // public static final double maxSpeed = Units.feetToMeters(15.1);
        public static final double maxSpeed = Units.feetToMeters(10);
        public static final double maxTurningSpeed = 4.5;

        public static final int frontLeftDriveID = 7;
        public static final int frontLeftSteerID = 1;
        public static final int frontLeftCANCoderID = 21;
        public static final double frontLeftEncoderOffset = 0.498535 -.25;

        public static final int frontRightDriveID = 8;
        public static final int frontRightSteerID = 6;
        public static final int frontRightCANCoderID = 23;
        public static final double frontRightEncoderOffset = 0.000977 - 0.25;

        public static final int backLeftDriveID = 3;
        public static final int backLeftSteerID = 4;
        public static final int backLeftCANCoderID = 20;
        public static final double backLeftEncoderOffset = -0.497803 + 0.75 ;

        public static final int backRightDriveID = 5;
        public static final int backRightSteerID = 2;
        public static final int backRightCANCoderID = 22;
        public static final double backRightEncoderOffset = 0.498535 - 0.25;

        public static final double xOffsetMeters = Units.inchesToMeters(12.5);
        public static final double yOffsetMeters = Units.inchesToMeters(12.5);

        public static final class ElevatorParams{
            public static final double kP = 0.3;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double kFF = 0;
            public static final double elevatorSpeed = 0.15;
            public static final int elevatorMotorID = 9;
            public static final double maxHeight = 26.0;
            public static final IdleMode kIdleMode = IdleMode.kBrake;
        }

        public static final class DriveParams {
            public static final double kP = 0.1/4.0;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double kFF = 1/473.0;
            public static final IdleMode kIdleMode = IdleMode.kBrake;
        }

        public static final class SteerParams {
            public static final double kP = 0.01;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double kFF = 0;
            public static final IdleMode kIdleMode = IdleMode.kBrake;
        }

        public static final class PitcherParams {
            public static final double pitcherSpeed = 0;
            public static final int pitcherMotorID = 12;
            public static final IdleMode kIdleMode = IdleMode.kBrake;
        }

        public static final class RollerParams {
            public static final double rollerSpeed = 0;
            public static final int rollerMotorID = 0; // TODO: Set motor ID
            public static final IdleMode kIdleMode = IdleMode.kBrake;
        }
    }
}
