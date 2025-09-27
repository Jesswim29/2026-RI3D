package frc.robot.controllers;

public interface DriveController {
    /**
     * Gives x value from -1 to 1 with 1 being forward
     */
    public double getDriveX();

    /**
     * Gives y value from -1 to 1 with 1 being forward
     */
    public double getDriveY();

    public double getRotation();
    public double getThrottle();
}
