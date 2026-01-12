package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ClimberParams;


public class Climb extends SubsystemBase {
    private final SparkMax LeftClimber; //SM 40
    private final SparkMax RightClimber;  //SM 41
    
    private final SparkClosedLoopController LeftPID;
    private final SparkClosedLoopController RightPID;

    public Climb(int leftID, int rightID){
        LeftClimber = new SparkMax(leftID, MotorType.kBrushless);
        RightClimber = new SparkMax(rightID, MotorType.kBrushless);

        LeftPID = LeftClimber.getClosedLoopController();
        RightPID = RightClimber.getClosedLoopController();

        configMotor();
    }




    private void configMotor(){
        var config = new SparkMaxConfig();

        config.encoder
            .positionConversionFactor(0)
            .velocityConversionFactor(0);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(
                ClimberParams.P,
                ClimberParams.I,
                ClimberParams.D
            );
    }
}