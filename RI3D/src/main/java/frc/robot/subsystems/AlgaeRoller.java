package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants.RollerParams;

public class AlgaeRoller extends SubsystemBase{
    private final SparkMax m_roller;
    private final RelativeEncoder m_internalEncoder;
    
    public AlgaeRoller() {
        m_roller = new SparkMax(RollerParams.rollerMotorID, MotorType.kBrushless);
        m_internalEncoder = m_roller.getEncoder();

        configMotor();

        m_internalEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
    }

    public void setVelocity(double velocity) {
        m_roller.set(velocity);
    }

    private void configMotor() {
        var config = new SparkMaxConfig();

        config.idleMode(RollerParams.kIdleMode);
        config.encoder // TODO: Setup the desired units we are using <---- DO NOT FORGET UNITS
        .positionConversionFactor((Units.inchesToMeters(4) * Math.PI))   // TODO: get diameter
        .velocityConversionFactor(((Units.inchesToMeters(4) * Math.PI) / 60)); // in meters per second
        // config.smartCurrentLimit(60, 30);
        config.inverted(true);

        m_roller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }    
}
