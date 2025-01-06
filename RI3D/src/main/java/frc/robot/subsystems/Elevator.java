package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants.ElevatorConstants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

// TODO
// - set up the PID loop
// - set up encoder conversions

public class Elevator extends SubsystemBase {
    private final SparkMax m_elevator;
    private final RelativeEncoder m_Encoder;
    
    public Elevator() {
        // TODO: Use trapezoid profile? 
        new ProfiledPIDController(0.3, 0, 0, new TrapezoidProfile.Constraints(3600 * ElevatorConstants.elevatorSpeed, 3000 * ElevatorConstants.elevatorSpeed));
        
        m_elevator = new SparkMax(ElevatorConstants.elevatorMotorID, MotorType.kBrushless);
        m_Encoder = m_elevator.getEncoder();

        // TODO set range and tolerance to the desired units we are using <---- DO NOT FORGET UNITS

    }

    /**
     * Set the position of the elevator
     * 
     * @param pos the position in inches
     */
    public void setPosition(double pos) {
        if (pos > ElevatorConstants.maxHeight)
        {
            System.out.printf("cannot set position to height %f - max height is %f!\n", pos, ElevatorConstants.maxHeight);
        }
        else
        {
            // TODO set position
        }
    }

    public double getPosition()
    {
        // TODO implement
        return 0d;
    }

    @Override
    public void periodic() {

    }

    private void configMotor() {
        // TODO
        var config = new SparkMaxConfig();

        config.alternateEncoder
            .positionConversionFactor(2 * Math.PI);

        m_elevator.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

}

