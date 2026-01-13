package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Launcher extends SubsystemBase {

    private final SparkMax leader;
    private final SparkClosedLoopController leadPID;
    final SimpleMotorFeedforward feedForward;

    public Launcher() {
        //Configuring Lead sparkmax motorcontroller
        leader = new SparkMax(
            Constants.LauncherConstants.flywheel1,
            MotorType.kBrushless
        );
        leadPID = leader.getClosedLoopController();

        //Setting feedforwardrate
        feedForward = new SimpleMotorFeedforward(0.12, .473);

        //Feed/Indexer/IDK motor
        configMotors();
    }

    private void configMotors() {
        var leadConfig = new SparkMaxConfig();
        //TODO find kP, kI, kD controls dunno if drivetrain constants count? as well as the rest of the values
        leadConfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);

        leadConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(
                Constants.LauncherConstants.flykP,
                Constants.LauncherConstants.flykI,
                Constants.LauncherConstants.flykD
            );

        leader.configure(
            leadConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    public double getVelocity() {
        //TODO return speed for flywheel
        return leader.getEncoder().getVelocity();
    }

    public void setVelocity(double speed) {
        leadPID.setReference(
            speed,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            feedForward.calculate(speed)
        );
        leader.set(-speed);
    }

    public void stopMotors() {
        leader.stopMotor();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
