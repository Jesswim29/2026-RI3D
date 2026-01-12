package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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
import frc.robot.Constants;
import frc.robot.Constants.ClimberParams;

/* TODO
 funcs
 * Extend arm (cases per arm)
 * Resist gravity (cases per arm)
 * Pullup (both)
 logic
 *>If first extension, other arm should not move! ->
 *>Else, other arm should resist gravity ->
 *UNTIL extended arm is in pos.
 *Then, other arm extends as the first resists gravity in ext.d pos. until also in ext.d pos.
 *Then, both arms retract until in origin pos & resisting gravity
 *^LOOP
 */

public class Climb extends SubsystemBase {
    private final SparkMax leftMotor; //SM 40
    private final SparkMax rightMotor;  //SM 41
    
    private SparkClosedLoopController leftPID, rightPID;
    private final RelativeEncoder leftEncoder, rightEncoder;
    private final DutyCycleEncoder leftAbsEncoder, rightAbsEncoder;


    public Climb(int leftID, int rightID){
        leftMotor = new SparkMax(Constants.ClimberParams.leftClimber, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.ClimberParams.rightClimber, MotorType.kBrushless);
        leftAbsEncoder = new DutyCycleEncoder(0);
        rightAbsEncoder = new DutyCycleEncoder(0);

        leftPID = LeftClimber.getClosedLoopController();
        rightPID = RightClimber.getClosedLoopController();

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        configMotor();
    }


    public void extendClimber(int ID){
        if(ID == Constants.ClimberParams.leftClimber){
            
        }
        else if(ID == Constants.ClimberParams.rightClimber){
            
        }
        else
            throw new IllegalArgumentException();
    }


    public double getAbsoluteClimberPos(int ID){
        if(ID == Constants.ClimberParams.leftClimber){
            return leftAbsEncoder.get();
        }
        else if(ID == Constants.ClimberParams.rightClimber){
            return rightAbsEncoder.get();
        }
        else
            throw new IllegalArgumentException();
    }

    public double getClimberPos(int ID){
        if(ID == Constants.ClimberParams.leftClimber){
            return leftEncoder.getPosition();
        }
        else if(ID == Constants.ClimberParams.rightClimber){
            return rightEncoder.getPosition();
        }
        else
            throw new IllegalArgumentException();
    }

    /** 
     * @param height encoder val for pos
     */
    public double setClimberPos(int ID, double height){
        if(ID == Constants.ClimberParams.leftClimber){
            leftPID.setReference(height, ControlType.kPosition);
        }
        else if(ID == Constants.ClimberParams.rightClimber){
            rightPID.setReference(height, ControlType.kPosition);
        }
        else
            throw new IllegalArgumentException();
    }

    /** 
     * @param percent percent -1 to 1 to run the motor at
     */
    public void setClimberSpeed(double percent){
        leftMotor.set(percent);
        rightMotor.set(percent);
    }

    public void periodic() {
    }

    private void configMotor(){
        var config = new SparkMaxConfig();

        config.encoder
            .positionConversionFactor(0)
            .velocityConversionFactor(0);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(
                ClimberParams.P,
                ClimberParams.I,
                ClimberParams.D,
                ClimberParams.FF
            );
    }
}