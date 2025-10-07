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

    public static final int pigeonID = 13;
    public static final int kButtonController = 1;
    public static final int kDriveController = 0;
    public static final double kDeadzone = 0.05;

    public static final class DrivetrainConstants {

        // public static final double maxSpeed = Units.feetToMeters(15.1);
        public static final double maxSpeed = Units.feetToMeters(25);

        public static final int backLeftDriveID = 1;
        public static final int backLeftSteerID = 2;
        public static final int backLeftCANCoderID = 9;
        public static final double backLeftEncoderOffset = 0.001709;

        public static final int frontLeftDriveID = 3;
        public static final int frontLeftSteerID = 4;
        public static final int frontLeftCANCoderID = 10;
        public static final double frontLeftEncoderOffset = 0.992920;

        public static final int frontRightDriveID = 5;
        public static final int frontRightSteerID = 6;
        public static final int frontRightCANCoderID = 11;
        public static final double frontRightEncoderOffset = 0.003418;

        public static final int backRightDriveID = 7;
        public static final int backRightSteerID = 8;
        public static final int backRightCANCoderID = 12;
        public static final double backRightEncoderOffset = 0.490234;

        public static final double xOffsetMeters = Units.inchesToMeters(12.5);
        public static final double yOffsetMeters = Units.inchesToMeters(12.5);

        public static final int stallLimit = 60;
        public static final int freeLimit = 20;

        public static final class DriveParams {

            public static final double kP = 0.1 / 4.0;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double kFF = 1 / 473.0;
            public static final IdleMode kIdleMode = IdleMode.kBrake;

            public static final double wheelRadius = 2; //in inches
            public static final double wheelDiameter = wheelRadius * 2;
            public static final double gearRatio = 6.75;
            public static final double positionConversionFactor = wheelDiameter * Math.PI / gearRatio; //gear ratio of SDS mk4 L2
            public static final double velocityConversionFactor = positionConversionFactor / 60;
        }

        public static final class SteerParams {

            public static final double kP = 0.025;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double kFF = 0;
            public static final IdleMode kIdleMode = IdleMode.kBrake;

            //ask andrew for story time if you really want to know (YOU DONT) (TRUST ME JUST LEAVE IT BE) 🦀
            public static final double steerConversionFactor = 28.13;
        }
    }
}
