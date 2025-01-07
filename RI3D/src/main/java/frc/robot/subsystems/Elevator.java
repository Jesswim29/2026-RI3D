package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.DrivetrainConstants.ElevatorConstants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

// TODO
// - set up the PID loop
// - set up encoder conversions

public class Elevator extends SubsystemBase {
    private final SparkMax m_elevator;
    private final RelativeEncoder m_internalEncoder;
    private final RelativeEncoder m_externalEncoder;
    private final ProfiledPIDController m_controller;
    
    public Elevator() {
        // TODO: Tune trapezoid profile
        m_controller = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, new TrapezoidProfile.Constraints(3600*Math.PI/180 * ElevatorConstants.elevatorSpeed, 3000*Math.PI/180 * ElevatorConstants.elevatorSpeed));
        
        m_elevator = new SparkMax(ElevatorConstants.elevatorMotorID, MotorType.kBrushless);
        m_internalEncoder = m_elevator.getEncoder();
        m_externalEncoder = m_elevator.getAlternateEncoder();

        configMotor();

        // TODO set range and tolerance
        m_controller.enableContinuousInput(0, ElevatorConstants.maxHeight);
        // m_controller.setTolerance()

        m_internalEncoder.setPosition(0);
        m_externalEncoder.setPosition(0);
        
        m_controller.reset(0);
    }

    /**
     * Set the position of the elevator
     * 
     * @param pos the position in inches
     */
    public void setPositionGoal(double pos) {
        // TODO: Remove this check as redundance with enableContinuousInput?
        if (pos > ElevatorConstants.maxHeight)
        {
            System.out.printf("cannot set position to height %f - max height is %f!\n", pos, ElevatorConstants.maxHeight);
        }
        else
        {
            // TODO set position
            m_controller.setGoal(pos);
        }
    }

    public double getPosition()
    {
        // TODO implement
        return m_internalEncoder.getPosition();
    }

    @Override
    public void periodic() {
        m_elevator.set(m_controller.calculate(m_internalEncoder.getPosition(),m_controller.getSetpoint()));
    }

    private void configMotor() {
        var config = new SparkMaxConfig();

        config.idleMode(DrivetrainConstants.DriveParams.kIdleMode);
        config.encoder // TODO: Setup the desired units we are using <---- DO NOT FORGET UNITS
            .positionConversionFactor((((Units.inchesToMeters(1.9) * Math.PI) / 16)))
            .velocityConversionFactor((((Units.inchesToMeters(1.9) * Math.PI) / 16) / 60)); // in meters per second
        config.alternateEncoder
            .positionConversionFactor((((Units.inchesToMeters(0.5) * Math.PI) / 16)))
            .velocityConversionFactor((((Units.inchesToMeters(0.5) * Math.PI) / 16) / 60)); // in meters per second
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder) // TODO: Could use alternate encoder?
            .pidf(
                ElevatorConstants.kP,
                ElevatorConstants.kI,
                ElevatorConstants.kD,
                ElevatorConstants.kFF
            );
        // config.smartCurrentLimit(60, 30);
        config.inverted(false);

        m_elevator.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

}

