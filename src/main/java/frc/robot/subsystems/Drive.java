package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class Drive extends SubsystemBase {

    public Wheel[] wheels;

    private double restAngle = 0;
    private double normalizingFactor = 1;
    private final Translation2d frontLeftLocation, frontRightLocation;
    private final Translation2d backLeftLocation, backRightLocation;

    private final Gyro m_Gyro;

    public Drive(Gyro gyro) {
        m_Gyro = gyro;

        // locations are also in terms of the wpilib coordinate system
        frontLeftLocation = new Translation2d(
            DrivetrainConstants.yOffsetMeters, //-
            -DrivetrainConstants.xOffsetMeters
        );
        frontRightLocation = new Translation2d(
            DrivetrainConstants.yOffsetMeters, //-
            DrivetrainConstants.xOffsetMeters
        );
        backLeftLocation = new Translation2d(
            -DrivetrainConstants.yOffsetMeters, //+
            -DrivetrainConstants.xOffsetMeters
        );
        backRightLocation = new Translation2d(
            -DrivetrainConstants.yOffsetMeters, //+
            DrivetrainConstants.xOffsetMeters
        );

        wheels = new Wheel[] {
            /* front left */
            new Wheel(
                frontLeftLocation,
                DrivetrainConstants.frontLeftDriveID,
                DrivetrainConstants.frontLeftSteerID,
                DrivetrainConstants.frontLeftCANCoderID,
                DrivetrainConstants.frontLeftEncoderOffset,
                true
            ),
            /* front right */
            new Wheel(
                frontRightLocation,
                DrivetrainConstants.frontRightDriveID,
                DrivetrainConstants.frontRightSteerID,
                DrivetrainConstants.frontRightCANCoderID,
                DrivetrainConstants.frontRightEncoderOffset,
                false
            ),
            /* back left */
            new Wheel(
                backLeftLocation,
                DrivetrainConstants.backLeftDriveID,
                DrivetrainConstants.backLeftSteerID,
                DrivetrainConstants.backLeftCANCoderID,
                DrivetrainConstants.backLeftEncoderOffset,
                true
            ),
            /* back right */
            new Wheel(
                backRightLocation,
                DrivetrainConstants.backRightDriveID,
                DrivetrainConstants.backRightSteerID,
                DrivetrainConstants.backRightCANCoderID,
                DrivetrainConstants.backRightEncoderOffset,
                false
            ),
        };
    }

    /**
     *
     * @param translation   linear movement (meters/sec)
     * @param rotation      rotational magnitude (radians/sec)
     */
    public void swerve(
        double linearAngle,
        double linearSpeed,
        double rotation,
        Translation2d rotationPoint
    ) {
        double[] calcWheelAngle = new double[wheels.length];
        double[] calcWheelSpeed = new double[wheels.length];
        double normalizingFactor = 1;

        if (linearAngle == 0 && linearSpeed == 0 && rotation == 0) {
            for (Wheel wheel : wheels) {
                wheel.stopMotors();
            }
            return;
        }

        int i = 0;
        for (Wheel wheel : wheels) {
            double wheelX = wheel.location.getX();
            double wheelY = wheel.location.getY();

            double deltaX = wheelX - rotationPoint.getX();
            double deltaY = wheelY - rotationPoint.getY();

            Translation2d linearVector = getLinearVector(
                linearSpeed,
                linearAngle
            );

            double rotationAngle = getRotationAngle(
                deltaX,
                deltaY,
                linearAngle,
                rotation
            );

            Translation2d rotationVector = getRotationVector(
                deltaX,
                deltaY,
                rotationAngle,
                rotation
            );
            Translation2d resultXY = new Translation2d(
                linearVector.getX() + rotationVector.getX(),
                linearVector.getY() + rotationVector.getY()
            );
            double resultSpeed = Math.sqrt(
                Math.pow(resultXY.getX(), 2) + Math.pow(resultXY.getY(), 2)
            );
            double resultAngle = getResultAngle(resultXY);
            calcWheelAngle[i] = resultAngle;
            calcWheelSpeed[i] = resultSpeed;

            // TODO: look into why this is needed more later
            if (resultSpeed > normalizingFactor) {
                normalizingFactor = resultSpeed;
            }

            i += 1;
        }

        i = 0;
        for (Wheel wheel : wheels) {
            SwerveModuleState inputState = new SwerveModuleState(
                DrivetrainConstants.maxSpeed *
                    (calcWheelSpeed[i] / normalizingFactor),
                Rotation2d.fromDegrees(calcWheelAngle[i])
            );

            SwerveModuleState optimizedState = wheel.bestAngle(inputState);
            System.out.println(
                "calc " +
                    calcWheelAngle[i] +
                    " opt " +
                    optimizedState.angle.getDegrees()
            );
            System.out.println();
            wheel.setAngle(optimizedState.angle.getDegrees());
            wheel.setSpeed(optimizedState.speedMetersPerSecond);

            i += 1;
        }
    }

    public void swerve(
        double linearAngle,
        double linearSpeed,
        double rotation
    ) {
        swerve(linearAngle, linearSpeed, rotation, new Translation2d(0, 0));
    }

    public Translation2d getLinearVector(
        double linearSpeed,
        double linearAngle
    ) {
        double linDeltaY = Math.cos(Math.toRadians(linearAngle)) * linearSpeed;
        double linDeltaX = Math.sin(Math.toRadians(linearAngle)) * linearSpeed;
        return new Translation2d(linDeltaX, linDeltaY);
    }

    public double getRotationAngle(
        double deltaX,
        double deltaY,
        double linearAngle,
        double rotation
    ) {
        // alpha is the angle of the vector to the wheel location
        // with respect forward and in degrees
        double alpha = 0;
        double rotationAngle = 0.0;

        if (deltaY == 0 && deltaX > 0) {
            // If the wheel is aligned with the center of rotation on the Y axis and
            // positive on the X axis alpha = 90
            alpha = 90;
        } else if (deltaY == 0 && deltaX < 0) {
            // If the wheel is aligned with the center of rotation on the Y axis and
            // negative on the X axis alpha = -90
            alpha = -90;
        } else if (deltaY == 0 && deltaX == 0) {
            // If the wheel is aligned with the center of rotation on both axis the alpha
            // should be the linear angle
            alpha = linearAngle;
        } else {
            // Otherwise use atan2 of delta X and delta Y to find angle
            alpha = Math.toDegrees(Math.atan2(deltaX, deltaY));
        }

        if (rotation > 0) {
            // If rotating clockwise add 90 to alpha to get angle
            rotationAngle = alpha + 90;
        } else if (rotation < 0) {
            // If rotating counter clockwise subtract 90 from alpha to get angle
            rotationAngle = alpha - 90;
        }

        return rotationAngle;
    }

    public Translation2d getRotationVector(
        double deltaX,
        double deltaY,
        double rotationAngle,
        double rotation
    ) {
        double rotationalSpeedY =
            Math.cos(Math.toRadians(rotationAngle)) * Math.abs(rotation);
        double rotationalSpeedX =
            Math.sin(Math.toRadians(rotationAngle)) * Math.abs(rotation);
        if (deltaX == 0 && deltaY == 0) {
            rotationalSpeedX = 0;
            rotationalSpeedY = 0;
        }
        return new Translation2d(rotationalSpeedX, rotationalSpeedY);
    }

    public double getResultAngle(Translation2d resultXY) {
        double resultAngle = Math.toDegrees(
            Math.atan2(resultXY.getY(), resultXY.getX())
        );
        resultAngle = -(90 - resultAngle);

        if (resultXY.getY() == 0 && resultXY.getX() == 0) {
            resultAngle = restAngle;
        } else {
            restAngle = resultAngle;
        }
        return resultAngle;
    }

    public void lockWheels() {
        for (int i = 0; i < wheels.length; i++) {
            wheels[i].setAngle((i + 45) * 90);
        }
    }

    public void goToAngle(double ang) {
        // TODO: impl
        /**
        for (SwerveModule curMod : wheels) {
           curMod.setDesiredState(
                new SwerveModuleState(0d, Rotation2d.fromDegrees(ang))
            );
        }
        */
    }
}
