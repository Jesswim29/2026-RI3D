package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;


public class Feed extends SubsystemBase{
    private final SparkMax Feed;

    public Feed(){
        //Configuring motors me tinks
        Feed = new SparkMax(Constants.LauncherConstants.feeder, MotorType.kBrushless);
        
        var FeedConfig = new SparkMaxConfig();

        FeedConfig.encoder.positionConversionFactor(0).velocityConversionFactor(0);


    }
    public void Activate() {
        Feed.set(.25);
    }
    public void stopMotors() {
        Feed.stopMotor();
    }
    @Override
    public void periodic() {
        //This method will be called once per scheduler run
    }
    
}
