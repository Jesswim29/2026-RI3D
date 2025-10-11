package frc.robot.subsystems;

public interface IGyro {

    void resetGyro();

    // sets the gyro offset in degrees
    void setGyroOffset(double offset);

    // gets the gyro offset in degrees
    double getGyroOffset();

    // gets the raw angle of the gyro (degrees rotated from start NOT 0-360)
    double getRawGyroAngle();

    // gets the real 0-360 angle of the robot relative to start
    double getRealGyroAngle();

    double getPitch();

    double getYaw();

    double getRoll();
}