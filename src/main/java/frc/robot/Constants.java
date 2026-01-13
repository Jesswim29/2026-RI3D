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
 * @param maxSpeed The maximum linear speed of a wheel
 */
public final class Constants {

    public static final int pigeonID = 13;
    public static final int kOperatorController = 1;
    public static final int kDriveController = 0;
    public static final double kDeadzone = 0.05;

    public static final class DrivetrainConstants {

        // public static final double maxSpeed = Units.feetToMeters(15.1);
        // public static final double maxSpeed = Units.feetToMeters(25);
        public static final double maxSpeed = 25;

        public static final int backLeftDriveID = 1;
        public static final int backLeftSteerID = 2;
        public static final int backLeftCANCoderID = 9;
        public static final double backLeftEncoderOffset = 0.001465; //.001709

        public static final int frontLeftDriveID = 3;
        public static final int frontLeftSteerID = 4;
        public static final int frontLeftCANCoderID = 10;
        public static final double frontLeftEncoderOffset = 0.992164; //.992920

        public static final int frontRightDriveID = 5;
        public static final int frontRightSteerID = 6;
        public static final int frontRightCANCoderID = 11;
        public static final double frontRightEncoderOffset = 0.004150; //.003418

        public static final int backRightDriveID = 7;
        public static final int backRightSteerID = 8;
        public static final int backRightCANCoderID = 12;
        public static final double backRightEncoderOffset = 0.4920; //.490234

        public static final double xOffsetMeters = Units.inchesToMeters(12.5);
        public static final double yOffsetMeters = Units.inchesToMeters(12.5);

        public static final int stallLimit = 60;
        public static final int freeLimit = 40;

        public static final class DriveParams {

            public static final double kP = 0.0000075; //.1/4.0
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double kFF = 1.0 / 473; //1/473
            public static final IdleMode kIdleMode = IdleMode.kBrake;

            public static final double wheelRadius = 2; //in inches
            public static final double wheelDiameter = wheelRadius * 2;
            public static final double gearRatio = 6.75;
            public static final double positionConversionFactor =
                (wheelDiameter * Math.PI) / gearRatio; //gear ratio of SDS mk4 L2
            public static final double velocityConversionFactor =
                positionConversionFactor / 60;
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

    public static class ClimberParams {

        public static final int leftID = 40;
        public static final int rightID = 41;
        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;
        public static final double FF = 0;
        //public static final double maxHeight = ;
    }

    public static class LauncherConstants {

        public static final int flywheel1 = 20;
        public static final int flywheel2 = 21;
        public static final int feeder = 22;
    }

    public static class IntakeConstants {

        public static final int intakePivot = 30;
        public static final int intakeRoller = 31;
        public static final int intakeConversionFactor = 25;
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kFF = 0;
    }

    //no touch please or bad things happen
    public static class DriveCommandConstants {

        public static final double P = 0.024; //.0024
        public static final double I = 0;
        public static final double D = 0;

        public static final double P2 = .04;
        public static final double I2 = .00012;
    }
}
