package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;

public class Intake extends SubsystemBase{
    private final SparkMax pivotMotor, rollerMotor;
    private final DutyCycleEncoder pivotAbsEncoder;
    private final RelativeEncoder pivotEncoder;
    private SparkClosedLoopController pivotPID;


    public Intake(){
        pivotMotor = new SparkMax(Constants.IntakeConstants.intakePivot, MotorType.kBrushless);
        rollerMotor = new SparkMax(Constants.IntakeConstants.intakeRoller, MotorType.kBrushless);

        pivotEncoder = pivotMotor.getEncoder();
        pivotAbsEncoder = new DutyCycleEncoder(0); //TODO pick port number

        pivotPID = pivotMotor.getClosedLoopController();

        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        SparkMaxConfig rollerConfig = new SparkMaxConfig();

        pivotConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20,20);
        pivotConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pidf(
                Constants.IntakeConstants.kP, 
                Constants.IntakeConstants.kI,
                Constants.IntakeConstants.kD,
                Constants.IntakeConstants.kFF
            );
        pivotConfig.encoder
            .positionConversionFactor(
                Constants.IntakeConstants.intakeConversionFactor
            )
            .velocityConversionFactor(
                Constants.IntakeConstants.intakeConversionFactor
            );
        
        rollerConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(20,20);

        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    } 

    /** 
     * @param percent percent -1 to 1 to run the motor at
     */
    public void setRollerSpeed(double percent){
        rollerMotor.set(percent);
    }

    public void stopRoller(){
        rollerMotor.stopMotor();
    }

    public void setPivotPos(double angle){
        pivotPID.setReference(angle, ControlType.kPosition);
    }

    public double getAbsolutePivotPos(){
        return pivotAbsEncoder.get();
    }

    public double getPivotPos(){
        return pivotEncoder.getPosition();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
