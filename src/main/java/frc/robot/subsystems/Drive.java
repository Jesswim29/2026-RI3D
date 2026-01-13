package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.gyros.Gyro;

public class Drive extends SubsystemBase {

    public Wheel[] wheels;

    private double restAngle = 0;
    private final Translation2d frontLeftLocation, frontRightLocation;
    private final Translation2d backLeftLocation, backRightLocation;

    // Remove the metadata if this variable actually gets used
    // I am keeping it in as it will probably be useful during Ri3D
    @SuppressWarnings("unused")
    private final Gyro gyro;

    public Drive(Gyro gyro) {
        this.gyro = gyro;

        // These are **not** in wpilib's coordinate system
        // in that we are XY instead of YX
        backLeftLocation = new Translation2d(
            -DrivetrainConstants.xOffsetMeters,
            -DrivetrainConstants.yOffsetMeters
        );
        frontLeftLocation = new Translation2d(
            -DrivetrainConstants.xOffsetMeters,
            DrivetrainConstants.yOffsetMeters
        );
        frontRightLocation = new Translation2d(
            DrivetrainConstants.xOffsetMeters,
            DrivetrainConstants.yOffsetMeters
        );
        backRightLocation = new Translation2d(
            DrivetrainConstants.xOffsetMeters,
            -DrivetrainConstants.yOffsetMeters
        );

        wheels = new Wheel[] {
            /* back left */
            new Wheel(
                backLeftLocation,
                DrivetrainConstants.backLeftDriveID,
                DrivetrainConstants.backLeftSteerID,
                DrivetrainConstants.backLeftCANCoderID,
                DrivetrainConstants.backLeftEncoderOffset,
                true
            ),
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
     * Moves robot using swerve drive
     * @param linearAngle   angle (degrees)
     * @param linearSpeed   linear movement (meters/sec)
     * @param rotation      rotational magnitude (radians/sec)
     * @param rotationPoint point of rotation in 2d space, can be inside or outside bot (x, y in meters)
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
            //System.out.println(
            //    "calc " +
            //        calcWheelAngle[i] +
            //        " opt " +
            //        optimizedState.angle.getDegrees()
            //);
            //SmartDashboard.putNumber(
            //    "Wheel " + i,
            //    optimizedState.angle.getDegrees()
            //);
            wheel.setAngle(optimizedState.angle.getDegrees());
            wheel.setSpeed(optimizedState.speedMetersPerSecond);

            i += 1;
        }
    }

    /**
     * Swerves robort provided Angle, speed and rotation
     * @param linearAngle   angle (degrees)
     * @param linearSpeed   linear movement (meters/sec)
     * @param rotation      rotational magnitude (radians/sec)
     */
    public void swerve(
        double linearAngle,
        double linearSpeed, //TODO: find difference between linearSpeed and rotation
        double rotation
    ) {
        swerve(linearAngle, linearSpeed, rotation, new Translation2d(0, 0));
    }

    /**
     * Locks the wheels in place duh
     */
    public void lockWheels() {
        for (int i = 0; i < wheels.length; i++) {
            wheels[i].setAngle((i + 45) * 90);
        }
    }

    /**
     * Get the Linear vector for the magnitude and direction
     * @param linearSpeed magnitude of the vector
     * @param linearAngle direction of the vector
     * @return
     */
    Translation2d getLinearVector(double linearSpeed, double linearAngle) {
        double linDeltaY = Math.cos(Math.toRadians(linearAngle)) * linearSpeed;
        double linDeltaX = Math.sin(Math.toRadians(linearAngle)) * linearSpeed;
        return new Translation2d(linDeltaX, linDeltaY);
    }

    /**
     * Getting the rotation angle from the change in x and y components
     * @param deltaX The change in the X component
     * @param deltaY The change in the y component
     * @param linearAngle The direction for the wheels to be aligned (I think)
     * @param rotation The magnitude or sped of rotation (I tink again)
     * @return The angle of rotation
     */
    double getRotationAngle(
        double deltaX,
        double deltaY,
        double linearAngle,
        double rotation
    ) {
        // Alpha is the angle of the vector to the wheel location
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

    /**
     * Probable for rotating the robot in place via the left joystick
     * @param deltaX Change in X component
     * @param deltaY Change in Y component
     * @param rotationAngle Direction of wheel to be aligned
     * @param rotation Magnitude of wheel
     * @return A translation2D object comprised of the x and y component
     */
    Translation2d getRotationVector(
        double deltaX,
        double deltaY,
        double rotationAngle,
        double rotation
    ) {
        //Calculating the rotationanal speed along the Y-axis
        double rotationalSpeedY =
            Math.cos(Math.toRadians(rotationAngle)) * Math.abs(rotation);
        //Calculating the rotational speed along the X-axis
        double rotationalSpeedX =
            Math.sin(Math.toRadians(rotationAngle)) * Math.abs(rotation);

        //If the difference between new and old X and Y coordinates is zero, then there is no knew position to calculate
        if (deltaX == 0 && deltaY == 0) {
            rotationalSpeedX = 0;
            rotationalSpeedY = 0;
        }
        // Translation2d object is comprised of an double x, and double y
        return new Translation2d(rotationalSpeedX, rotationalSpeedY);
    }

    /**
     * Converts the translation2d object using black magic witch craft
     * @param resultXY god knows what this is for
     * @return the resulting angle
     */
    double getResultAngle(Translation2d resultXY) {
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

    public Wheel[] getWheels() {
        return wheels;
    }
}
