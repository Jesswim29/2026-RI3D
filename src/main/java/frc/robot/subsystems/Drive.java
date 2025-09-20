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

    private SwerveModule[] m_mods;

    private final Translation2d m_frontLeftLocation, m_frontRightLocation;
    private final Translation2d m_backLeftLocation, m_backRightLocation;
    private final SwerveDriveKinematics m_kinematics;

    private final Gyro m_Gyro;

    public Drive(Gyro gyro) {
        m_Gyro = gyro;

        m_mods = new SwerveModule[] {
            /* front left */
            new SwerveModule(
                DrivetrainConstants.frontLeftDriveID,
                DrivetrainConstants.frontLeftSteerID,
                DrivetrainConstants.frontLeftCANCoderID,
                DrivetrainConstants.frontLeftEncoderOffset,
                0
            ),
            /* front right */
            new SwerveModule(
                DrivetrainConstants.frontRightDriveID,
                DrivetrainConstants.frontRightSteerID,
                DrivetrainConstants.frontRightCANCoderID,
                DrivetrainConstants.frontRightEncoderOffset,
                1
            ),
            /* back left */
            new SwerveModule(
                DrivetrainConstants.backLeftDriveID,
                DrivetrainConstants.backLeftSteerID,
                DrivetrainConstants.backLeftCANCoderID,
                DrivetrainConstants.backLeftEncoderOffset,
                2
            ),
            /* back right */
            new SwerveModule(
                DrivetrainConstants.backRightDriveID,
                DrivetrainConstants.backRightSteerID,
                DrivetrainConstants.backRightCANCoderID,
                DrivetrainConstants.backRightEncoderOffset,
                3
            ),
        };

        // locations are also in terms of the wpilib coordinate system
        m_frontLeftLocation = new Translation2d(
            -DrivetrainConstants.yOffsetMeters,
            -DrivetrainConstants.xOffsetMeters
        );
        m_frontRightLocation = new Translation2d(
            -DrivetrainConstants.yOffsetMeters,
            DrivetrainConstants.xOffsetMeters
        );
        m_backLeftLocation = new Translation2d(
            DrivetrainConstants.yOffsetMeters,
            -DrivetrainConstants.xOffsetMeters
        );
        m_backRightLocation = new Translation2d(
            DrivetrainConstants.yOffsetMeters,
            DrivetrainConstants.xOffsetMeters
        );

        m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation,
            m_frontRightLocation,
            m_backLeftLocation,
            m_backRightLocation
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
        for (SwerveModule curMod : m_mods) {
            curMod.setDesiredState(
                new SwerveModuleState(0d, Rotation2d.fromDegrees(ang))
            );
        }
    }

    public void printPosition() {
        //System.out.println(m_frontLeft.getAbsEncoderPos());
        //System.out.println(m_frontLeft.getPosition());
    }
}
