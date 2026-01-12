package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.ctre.phoenix6.controls.Follower;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;



public class Launcher extends SubsystemBase {
    private final SparkMax Leader;
    private final SparkMax Follower;
    private final SparkClosedLoopController LeadPID;
    final SimpleMotorFeedforward feedForward;
    public Launcher(int Fly1, int Fly2) {

        //Configuring Lead sparkmax motorcontroller
        Leader = new SparkMax(Fly1, MotorType.kBrushless);
        
        LeadPID = Leader.getClosedLoopController();

        Follower = new SparkMax(Fly2, MotorType.kBrushless);
    
        feedForward = new SimpleMotorFeedforward(0.12, .473);
        

        
    }
    private void configMotors(){
        var LeadConfig = new SparkMaxConfig();
        //TODO find kP, kI, kD controls dunno if drivetrain constants count? as well as the rest of the values
        LeadConfig.encoder.positionConversionFactor(0)
        .velocityConversionFactor(0);

        LeadConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(0, 0, 0, 0);

        Leader.configure(LeadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);




        //Configure FollowerID to be a follower of leader sparkmax
        var FollowConfig = new SparkMaxConfig();

        FollowConfig.follow(Leader.getDeviceId(), false);
        Follower.configure(FollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    }

    public double getVelocity(){
        //TODO return speed for flywheel
        return Leader.getEncoder().getVelocity();

    }
    void setVelocity(double speed) {
        LeadPID.setReference(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedForward.calculate(speed));
    }


    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    



}