package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;

public class XboxDriveController implements DriveController {

    XboxController controller;

    public XboxDriveController(XboxController controller) {
        this.controller = controller;
    }

    public double getDriveX() {
        return -controller.getRightX();
    }

    public double getDriveY() {
        return -controller.getRightY();
    }

    public double getRotation() {
        return controller.getLeftX();
    }

    public double getThrottle() {
        return -controller.getLeftY();
    }
}
