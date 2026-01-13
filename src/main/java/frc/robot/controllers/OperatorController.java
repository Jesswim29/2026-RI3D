package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperatorController {
    public Trigger extendIntake();
    public Trigger retractIntake();
    public Trigger launch();
    public Trigger toggleFeeder();
    //To remove later:
    public Trigger leftArmUp();
    public Trigger leftArmDown();
    public Trigger rightArmUp();
    public Trigger rightArmDown();
}
