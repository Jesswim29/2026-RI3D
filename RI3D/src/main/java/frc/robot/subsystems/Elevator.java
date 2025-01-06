package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants.ElevatorConstants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Elevator extends ProfiledPIDSubsystem{
    private final SparkMax m_elevator;
    private final RelativeEncoder m_Encoder;
    
    public Elevator() {
        //TODO: Use trapezoid profile? 
        super(new ProfiledPIDController(0.3, 0, 0, new TrapezoidProfile.Constraints(3600 * ElevatorConstants.elevatorSpeed, 3000 * ElevatorConstants.elevatorSpeed)));
        
        m_elevator = new SparkMax(ElevatorConstants.elevatorMotorID, MotorType.kBrushless);

        m_Encoder = m_elevator.getEncoder();

        //TODO initiate sparkmax config
        //m_Encoder.setPositionConversionFactor((1/16.0) * 360) ; // TODO check gear ratio, currently 1:16, 
        m_Encoder.setPosition(0);

        //TODO set range and tolerance to the desired units we are using <---- DO NOT FORGET UNITS

        super.setGoal(0);
        this.enable();

    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) 
    {//Use the output (and optionally the setpoint) here
        m_elevator.setVoltage(output);
    }

    @Override
    public double getMeasurement() {
        //Return the process variable measurement here
        return m_Encoder.getPosition();
    }
    @Override
    public void periodic(){
        super.periodic();
    }

    
}

