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

    private final SparkMax Leader;
    private final SparkClosedLoopController LeadPID;
    final SimpleMotorFeedforward feedForward;

    public Launcher() {
        //Configuring Lead sparkmax motorcontroller
        Leader = new SparkMax(
            Constants.LauncherConstants.flywheel1,
            MotorType.kBrushless
        );
        LeadPID = Leader.getClosedLoopController();

        //Setting feedforwardrate
        feedForward = new SimpleMotorFeedforward(0.12, .473);

        //Feed/Indexer/IDK motor
        configMotors();
    }

    private void configMotors() {
        var LeadConfig = new SparkMaxConfig();
        //TODO find kP, kI, kD controls dunno if drivetrain constants count? as well as the rest of the values
        LeadConfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);

        LeadConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(
                Constants.LauncherConstants.flykP,
                Constants.LauncherConstants.flykI,
                Constants.LauncherConstants.flykD
            );

        Leader.configure(
            LeadConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    public double getVelocity() {
        //TODO return speed for flywheel
        return Leader.getEncoder().getVelocity();
    }

    public void setVelocity(double speed) {
        LeadPID.setReference(
            speed,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            feedForward.calculate(speed)
        );
        Leader.set(-speed);
    }

    public void stopMotors() {
        Leader.stopMotor();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
