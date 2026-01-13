package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants;
import frc.robot.Constants.ClimberParams;
import frc.robot.REVMagneticLimit;

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
    private final SparkMax rightMotor; //SM 41

    // REVMagneticLimit limitLeft;
    // REVMagneticLimit limitRight;

    private SparkClosedLoopController leftPID, rightPID;
    private final RelativeEncoder leftEncoder, rightEncoder;
    private final DutyCycleEncoder leftAbsEncoder, rightAbsEncoder;

    public Climb() {
        leftMotor = new SparkMax(
            Constants.ClimberParams.leftID,
            MotorType.kBrushless
        );
        rightMotor = new SparkMax(
            Constants.ClimberParams.rightID,
            MotorType.kBrushless
        );
        leftAbsEncoder = new DutyCycleEncoder(5);
        rightAbsEncoder = new DutyCycleEncoder(6);

        leftPID = leftMotor.getClosedLoopController();
        rightPID = rightMotor.getClosedLoopController();

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        // limitLeft = new REVMagneticLimit(0);
        // limitRight = new REVMagneticLimit(1);

        configMotor();
    }

    public void extendClimber(int id) {
        if (id == Constants.ClimberParams.leftID) {
            leftMotor.set(0.2);
        } else if (id == Constants.ClimberParams.rightID) {
            rightMotor.set(0.2);
        } else throw new IllegalArgumentException();
    }

    public void retractClimber(int id) {
        if (id == Constants.ClimberParams.leftID) {
            leftMotor.set(-0.2);
        } else if (id == Constants.ClimberParams.rightID) {
            rightMotor.set(-0.2);
        } else throw new IllegalArgumentException();
    }

    public double getAbsoluteClimberPos(int id) {
        if (id == Constants.ClimberParams.leftID) {
            return leftAbsEncoder.get();
        } else if (id == Constants.ClimberParams.rightID) {
            return rightAbsEncoder.get();
        } else throw new IllegalArgumentException();
    }

    public double getClimberPos(int id) {
        if (id == Constants.ClimberParams.rightID) {
            return leftEncoder.getPosition();
        } else if (id == Constants.ClimberParams.leftID) {
            return rightEncoder.getPosition();
        } else throw new IllegalArgumentException();
    }

    /**
     * @param height encoder val for pos
     */
    public void setClimberPos(int id, double height) {
        if (id == Constants.ClimberParams.leftID) {
            leftPID.setReference(height, ControlType.kPosition);
        } else if (id == Constants.ClimberParams.rightID) {
            rightPID.setReference(height, ControlType.kPosition);
        } else throw new IllegalArgumentException();
    }

    /**
     * @param percent percent -1 to 1 to run the motor at
     */
    public void setClimberSpeed(double percent) {
        leftMotor.set(percent);
        rightMotor.set(percent);
    }

    public void stopClimber(int id) {
        if (id == Constants.ClimberParams.leftID) {
            leftMotor.stopMotor();
        } else if (id == Constants.ClimberParams.rightID) {
            rightMotor.stopMotor();
        } else throw new IllegalArgumentException();
    }

    public void periodic() {}

    private void configMotor() {
        var config = new SparkMaxConfig();

        config.encoder.positionConversionFactor(0).velocityConversionFactor(0);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(
                ClimberParams.P,
                ClimberParams.I,
                ClimberParams.D,
                ClimberParams.FF
            );
    }

    public void setPosition(SparkClosedLoopController PID, double distance) {
        //Sets the position of either motor to the specified distance
        PID.setReference(distance, ControlType.kPosition);
    }

    public double getPosition(SparkMax motor) {
        return motor.getEncoder().getPosition();
    }

    public void resetEncoders() {}

    public void stopMotors(SparkMax motor) {
        motor.set(0);
    }
}
