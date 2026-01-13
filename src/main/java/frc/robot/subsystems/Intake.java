package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.math.controller.PIDController;

public class Intake extends SubsystemBase{
    private final SparkMax pivotMotor, rollerMotor;
    private final DutyCycleEncoder pivotAbsEncoder;
    private RelativeEncoder pivotEncoder;
    // private SparkClosedLoopController pivotPID;


    public Intake(){
        pivotMotor = new SparkMax(Constants.IntakeConstants.intakePivot, MotorType.kBrushless);
        rollerMotor = new SparkMax(Constants.IntakeConstants.intakeRoller, MotorType.kBrushless);

        // pivotPID = pivotMotor.getClosedLoopController();
        
        
        pivotEncoder = pivotMotor.getEncoder();

        pivotAbsEncoder = new DutyCycleEncoder(2); //TODO pick port number

        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        SparkMaxConfig rollerConfig = new SparkMaxConfig();

        pivotConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20,20)
            .inverted(false);
        // pivotConfig.closedLoop
        //     .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        //     .pidf(
        //         Constants.IntakeConstants.kP,  //TODO PID Values
        //         Constants.IntakeConstants.kI,
        //         Constants.IntakeConstants.kD,
        //         Constants.IntakeConstants.kFF
        //     );
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

        // initalize();
    } 

    public void initalize(){
        double startPos = getAbsolutePivotPos(); //TODO constant and explain this later
        pivotEncoder.setPosition((startPos - 32) * 360);
    }

    /** 
     * @param percent percent -1 to 1 to run the motor at
     */
    public void setRollerSpeed(double percent){
        rollerMotor.set(-percent);
        SmartDashboard.putNumber("roller percent: ", percent);
    }

    public void stopRoller(){
        rollerMotor.stopMotor();
    }

    public void setPivotPos(double angle){
        // pivotPID.setReference(angle, ControlType.kPosition);
        // SmartDashboard.putNumber("pivot angle: ", angle);
    }

    public void setPivotSpeed(double speed){
        pivotMotor.set(speed);
    }

    //mult by 360 to get it in degrees instead of rotation
    public double getAbsolutePivotPos(){
        return 360 * (pivotAbsEncoder.get());
    }

    public double getPivotPos(){
        return (pivotEncoder.getPosition());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("ABS pos (intake): ", getAbsolutePivotPos());
        SmartDashboard.putNumber("Pos non ABS (intake)", getPivotPos());
    }
}
