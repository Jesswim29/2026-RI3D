package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperatorController {
    public Trigger extendIntake();
    public Trigger retractIntake();
    public Trigger reverseIntake();
    public Trigger launch();
    public Trigger toggleFeeder();
}
