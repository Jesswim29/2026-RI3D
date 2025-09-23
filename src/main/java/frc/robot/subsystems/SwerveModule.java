package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class SwerveModule extends SubsystemBase {

    private final CANcoder canCoder;
    private final SparkMax driveMotor, steerMotor;

    private final RelativeEncoder driveEncoder, steerEncoder;

    private final SparkClosedLoopController drivePID;
    private final SparkClosedLoopController steerPID;

    public boolean inverted;

    public SwerveModule(
        final int driveID,
        final int steerID,
        final int encoderID,
        final double encoderOffset,
        int swerveID,
        boolean inverted
    ) {
        canCoder = new CANcoder(encoderID);
        driveMotor = new SparkMax(driveID, MotorType.kBrushless);
        steerMotor = new SparkMax(steerID, MotorType.kBrushless);

        drivePID = driveMotor.getClosedLoopController();
        steerPID = steerMotor.getClosedLoopController();

        driveEncoder = driveMotor.getEncoder();
        steerEncoder = steerMotor.getEncoder();

        this.inverted = inverted;

        configSteer();
        resetEncoders();
        configDrive();
    }

    private double getSpeed() {
        // TODO: IMPL
        return 0;
    }

    public void setSpeed() {
        // TODO: IMPL
        return;
    }

    public double getAngleAbsolute() {
        // TODO: IMPL
        return 0.0;
    }

    public double getAngleRelative() {
        // TODO: IMPL
        return 0.0;
    }

    private void resetEncoders() {
        driveEncoder.setPosition(0);
        steerEncoder.setPosition(getAngleAbsolute());
        steerPID.setReference(0, ControlType.kPosition);
        // this relies on drive feed forward, we killed that in the great code fire
        //drivePID.setReference(
        //    0,
        //    ControlType.kVelocity,
        //    ClosedLoopSlot.kSlot0,
        //    m_driveFF.calculate(0.5)
        //);
    }

    private void configDrive() {
        var config = new SparkMaxConfig();

        config
            .idleMode(DrivetrainConstants.DriveParams.kIdleMode)
            .smartCurrentLimit(60, 20)
            .inverted(inverted)
            .encoder.positionConversionFactor(
                (((Units.inchesToMeters(4) * Math.PI) / 6.75))
            )
            .velocityConversionFactor(
                (((Units.inchesToMeters(4) * Math.PI) / 6.75) / 60)
            ); // in meters per second

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(
                DrivetrainConstants.DriveParams.kP,
                DrivetrainConstants.DriveParams.kI,
                DrivetrainConstants.DriveParams.kD,
                DrivetrainConstants.DriveParams.kFF
            );

        driveMotor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    public void configSteer() {
        var config = new SparkMaxConfig();

        config.encoder.positionConversionFactor((1 / 12.8) * 2 * Math.PI);

        config.closedLoop.pid(0.5, 0, 0);

        steerMotor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        MagnetSensorConfigs encoderConfig = new MagnetSensorConfigs();
        canCoder.getConfigurator().refresh(encoderConfig);

        // make the CANCoder return a value (0,1) instead of (-0.5,0.5)
        encoderConfig.withAbsoluteSensorDiscontinuityPoint(1);
        canCoder.getConfigurator().apply(encoderConfig);
    }

    public void reZero() {
        // TODO: IMPL
    }
}
