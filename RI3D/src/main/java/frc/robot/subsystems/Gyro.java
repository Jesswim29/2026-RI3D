package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class Gyro {
    private Pigeon2 m_gyro = new Pigeon2(Constants.pigeonID);

    public Gyro() {
    }

    /**
     * 
     * @return raw gyro angle in degrees
     */
    public Rotation2d getGyroAngle() {
        return m_gyro.getRotation2d();
    }

    public void zeroGyro() {
        m_gyro.setYaw(0);
    }

    /**
     * 
     * @return gyro angle in degrees, clamped [0,360)
     */
    public double getGyroAngleClamped() {
        // TODO check out what this was for
        return m_gyro.getRotation2d().getDegrees();
    }
}
