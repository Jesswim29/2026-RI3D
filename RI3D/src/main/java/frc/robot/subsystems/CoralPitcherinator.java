package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants.PitcherParams;

public class CoralPitcherinator extends SubsystemBase{
    private final SparkMax m_pitcher;
    private final RelativeEncoder m_internalEncoder;
    
    public CoralPitcherinator() {
        m_pitcher = new SparkMax(PitcherParams.pitcherMotorID, MotorType.kBrushless);
        m_internalEncoder = m_pitcher.getEncoder();

        configMotor();

        m_internalEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
    }

    public void setVelocity(double velocity) {
        m_pitcher.set(velocity);
    }

    private void configMotor() {
        var config = new SparkMaxConfig();

        config.idleMode(PitcherParams.kIdleMode);
        config.encoder // TODO: Setup the desired units we are using <---- DO NOT FORGET UNITS
        .positionConversionFactor((((Units.inchesToMeters(4) * Math.PI) / 12)))   // TODO: get diameter
        .velocityConversionFactor((((Units.inchesToMeters(4) * Math.PI) / 12) / 60)); // in meters per second
        // config.smartCurrentLimit(60, 30);
        // config.inverted(false);

        m_pitcher.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }    
}
