package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxDriveController implements DriveController {

    XboxController controller;

    public XboxDriveController(XboxController controller) {
        this.controller = controller;
    }

    public double getDriveX() {
        return controller.getRightX();
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

    public Trigger reset() {
        return new Trigger(() -> false);
    }

    public Trigger leftArmUp() {
        return new Trigger(() -> false);
    }
    public Trigger leftArmDown() {
        return new Trigger(() -> false);
    }
    public Trigger rightArmUp() {
        return new Trigger(() -> false);
    }
    public Trigger rightArmDown() {
        return new Trigger(() -> false);
    }
}
