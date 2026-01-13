package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxOperatorController implements OperatorController {

    XboxController controller;

    public XboxOperatorController(XboxController controller) {
        this.controller = controller;
    }

    public Trigger extendIntake() {
        return new Trigger(() -> controller.getLeftTriggerAxis() > 0.5);
    }

    public Trigger retractIntake() {
        return new Trigger(() -> controller.getLeftTriggerAxis() < 0.5);
    }

    public Trigger reverseIntake() {
        return new Trigger(() -> controller.getAButton());
    }

    public Trigger launch() {
        return new Trigger(() -> controller.getYButton());
    }

    public Trigger toggleFeeder() {
        return new Trigger(() -> controller.getRightTriggerAxis() >= 0.5);
    }
}
