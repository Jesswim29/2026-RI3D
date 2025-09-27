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

    private Wheel[] wheels;

    private final Translation2d frontLeftLocation, frontRightLocation;
    private final Translation2d backLeftLocation, backRightLocation;
    private final SwerveDriveKinematics m_kinematics;

    private final Gyro m_Gyro;

    public Drive(Gyro gyro) {
        m_Gyro = gyro;

        wheels = new Wheel[] {
            /* front left */
            new Wheel(
                0,
                0, // x and y
                DrivetrainConstants.frontLeftDriveID,
                DrivetrainConstants.frontLeftSteerID,
                DrivetrainConstants.frontLeftCANCoderID,
                DrivetrainConstants.frontLeftEncoderOffset,
                true
            ),
            /* front right */
            new Wheel(
                0,
                0, // x and y
                DrivetrainConstants.frontRightDriveID,
                DrivetrainConstants.frontRightSteerID,
                DrivetrainConstants.frontRightCANCoderID,
                DrivetrainConstants.frontRightEncoderOffset,
                false
            ),
            /* back left */
            new Wheel(
                0,
                0, // x and y
                DrivetrainConstants.backLeftDriveID,
                DrivetrainConstants.backLeftSteerID,
                DrivetrainConstants.backLeftCANCoderID,
                DrivetrainConstants.backLeftEncoderOffset,
                true
            ),
            /* back right */
            new Wheel(
                0,
                0, // x and y
                DrivetrainConstants.backRightDriveID,
                DrivetrainConstants.backRightSteerID,
                DrivetrainConstants.backRightCANCoderID,
                DrivetrainConstants.backRightEncoderOffset,
                false
            ),
        };

        // locations are also in terms of the wpilib coordinate system
        frontLeftLocation = new Translation2d(
            -DrivetrainConstants.yOffsetMeters,
            -DrivetrainConstants.xOffsetMeters
        );
        frontRightLocation = new Translation2d(
            -DrivetrainConstants.yOffsetMeters,
            DrivetrainConstants.xOffsetMeters
        );
        backLeftLocation = new Translation2d(
            DrivetrainConstants.yOffsetMeters,
            -DrivetrainConstants.xOffsetMeters
        );
        backRightLocation = new Translation2d(
            DrivetrainConstants.yOffsetMeters,
            DrivetrainConstants.xOffsetMeters
        );

        m_kinematics = new SwerveDriveKinematics(
            frontLeftLocation,
            frontRightLocation,
            backLeftLocation,
            backRightLocation
        );
    }

    /**
     *
     * @param translation   linear movement (meters/sec)
     * @param rotation      rotational magnitude (radians/sec)
     * @param fieldOriented if true, swerve with respect to the bot
     */
    public void swerve(
        Translation2d translation,
        Double rotation,
        boolean fieldOriented
    ) {
        SmartDashboard.putBoolean("fieldOriented (the voices)", fieldOriented);
        ChassisSpeeds speeds;
        if (fieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                Rotation2d.fromDegrees(m_Gyro.getGyroAngleClamped())
            );
        } else {
            speeds = new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation
            );
        }

        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(
            speeds
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(
            moduleStates,
            DrivetrainConstants.maxSpeed
        );
    }

    /**
     * swerve with respect to the field
     * @param translation   linear movement (meters/sec)
     * @param rotation      rotation (radians/sec)
     */
    public void swerve(Translation2d translation, Double rotation) {
        swerve(translation, rotation, false);
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
