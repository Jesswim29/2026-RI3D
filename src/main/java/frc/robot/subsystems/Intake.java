package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    private final SparkMax pivotMotor, rollerMotor;
    private final DutyCycleEncoder pivotEncoder;

    public Intake(){
        pivotMotor = new SparkMax(Constants.IntakeConstants.intakePivot, MotorType.kBrushless);
        rollerMotor = new SparkMax(Constants.IntakeConstants.intakeRoller, MotorType.kBrushless);

        pivotEncoder = new DutyCycleEncoder(0);

        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        SparkMaxConfig rollerConfig = new SparkMaxConfig();

        pivotConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(10,20);
        pivotConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pidf(0,0,0,0);
    } 

    public void setRollerSpeed(){
        //TODO implement lol
    }

    public double getRollerSpeed(){
        return 0;
    }

    public void setPivotPos(){
        //TODO implement lol
    }

    public double getPivotPos(){
        return 0;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
