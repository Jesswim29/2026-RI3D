package frc.robot.controllers;

import frc.lib.SpektrumFlightController;

public class SpektrumDriveController implements DriveController{
    
    SpektrumFlightController controller;

    public SpektrumDriveController(SpektrumFlightController controller){
        this.controller = controller;
    }

    public double getDriveX() {
        return controller.getDriveX();
    }

    public double getDriveY() {
        return -controller.getDriveY();
    }

    public double getRotation() {
        return controller.getThrottle();
    }

    public double getThrottle() {
        return -controller.getRotate();
    }
}
