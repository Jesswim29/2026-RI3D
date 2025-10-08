package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class Wheel extends SubsystemBase {

    private final CANcoder canCoder;
    private final SparkMax driveMotor, steerMotor;

    private final RelativeEncoder driveEncoder, steerEncoder;

    private SparkClosedLoopController drivePID;
    private SparkClosedLoopController steerPID;

    final SimpleMotorFeedforward feedForward;

    public Translation2d location;
    double encoderOffset;
    public boolean inverted;

    public Wheel(
        Translation2d location,
        int driveID,
        int steerID,
        int encoderID,
        double encoderOffset,
        boolean inverted
    ) {
        canCoder = new CANcoder(encoderID);
        driveMotor = new SparkMax(driveID, MotorType.kBrushless);
        steerMotor = new SparkMax(steerID, MotorType.kBrushless);

        drivePID = driveMotor.getClosedLoopController();
        steerPID = steerMotor.getClosedLoopController();

        driveEncoder = driveMotor.getEncoder();
        steerEncoder = steerMotor.getEncoder();

        this.location = location;
        this.encoderOffset = encoderOffset;
        this.inverted = inverted;

        // ks is the minimum amount of volts to make the motor barely move, kv was gotten from #0 who got it from neo docs
        feedForward = new SimpleMotorFeedforward(0.12, .473);

        configSteer();
        resetEncoders();
        configDrive();

        zero();
    }

    private void configDrive() {
        var config = new SparkMaxConfig();

        config
            .idleMode(DrivetrainConstants.DriveParams.kIdleMode)
            .smartCurrentLimit(
                DrivetrainConstants.stallLimit,
                DrivetrainConstants.freeLimit
            )
            .inverted(inverted);

        config.encoder
            .positionConversionFactor(
                DrivetrainConstants.DriveParams.positionConversionFactor
            )
            .velocityConversionFactor(
                DrivetrainConstants.DriveParams.velocityConversionFactor
            );

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

        config
            .idleMode(DrivetrainConstants.DriveParams.kIdleMode)
            .smartCurrentLimit(
                DrivetrainConstants.stallLimit,
                DrivetrainConstants.freeLimit
            );

        config.encoder.positionConversionFactor(DrivetrainConstants.SteerParams.steerConversionFactor); 

        config.closedLoop.pid(
            DrivetrainConstants.SteerParams.kP, 
            DrivetrainConstants.SteerParams.kI, 
            DrivetrainConstants.SteerParams.kD
        );

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

    public double getSpeed() {
        return driveEncoder.getVelocity();
    }

    public double getDistance() {
        return driveEncoder.getPosition();
    }

    public void setSpeed(double speed) {
        drivePID.setReference(
            speed,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            feedForward.calculate(speed)
        );
        SmartDashboard.putNumber("Speed", speed);
    }

    public void setPosition(double position) {
        drivePID.setReference(position, ControlType.kPosition);
    }

    public void setAngle(double angle) {
        steerPID.setReference(angle, ControlType.kPosition);
        SmartDashboard.putNumber("Angle", angle);
    }

    private void resetEncoders() {
        driveEncoder.setPosition(0);
    }

    public SwerveModuleState bestAngle(SwerveModuleState state) {
        double currentAngle = steerMotor.getEncoder().getPosition();
        int totalRotations = (int) (currentAngle / 360);
        double wantedAngle = state.angle.getDegrees() + 180;

        double deltaCurrent =
            (wantedAngle + (360 * totalRotations)) - currentAngle;
        double deltaNext =
            (wantedAngle + (360 * (totalRotations + 1))) - currentAngle;
        double deltaPrevious =
            (wantedAngle + (360 * (totalRotations - 1))) - currentAngle;

        double complimentaryAngle = (wantedAngle + 180) % 360;
        double deltaCurrentComp =
            (complimentaryAngle + (360 * totalRotations)) - currentAngle;
        double deltaNextComp =
            (complimentaryAngle + (360 * (totalRotations + 1))) - currentAngle;
        double deltaPreviousComp =
            (complimentaryAngle + (360 * (totalRotations - 1))) - currentAngle;

        double smallestAngle = deltaPrevious;
        if (Math.abs(smallestAngle) > Math.abs(deltaCurrent)) {
            smallestAngle = deltaCurrent;
        }
        if (Math.abs(smallestAngle) > Math.abs(deltaNext)) {
            smallestAngle = deltaNext;
        }

        double smallestAngleComp = deltaPreviousComp;
        if (Math.abs(smallestAngleComp) > Math.abs(deltaCurrentComp)) {
            smallestAngleComp = deltaCurrentComp;
        }
        if (Math.abs(smallestAngleComp) > Math.abs(deltaNextComp)) {
            smallestAngleComp = deltaNextComp;
        }

        double finalSmallestAngle = 0;
        int direction = 0;
        if (Math.abs(smallestAngle) <= Math.abs(smallestAngleComp)) {
            finalSmallestAngle = smallestAngle;
            direction = -1;
        } else {
            finalSmallestAngle = smallestAngleComp;
            direction = 1;
        }

        return new SwerveModuleState(
            state.speedMetersPerSecond * direction,
            new Rotation2d(
                Units.degreesToRadians(currentAngle + finalSmallestAngle)
            )
        );
    }

    public void stopMotors() {
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }

    public void zero() {
        double initial = canCoder.getAbsolutePosition().getValueAsDouble();
        // MAAAAYBE converts initial angle into encoder ticks, not our problem
        double absolute = (initial - encoderOffset) * 360;
        steerMotor.getEncoder().setPosition(absolute);
    }
}
