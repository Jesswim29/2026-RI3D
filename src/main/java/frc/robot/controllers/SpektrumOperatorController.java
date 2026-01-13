package frc.robot.controllers;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SpektrumOperatorController
    extends GenericHID
    implements OperatorController
    {
        
    public SpektrumOperatorController(int port) {
        super(port);
        HAL.report(tResourceType.kResourceType_XboxController, port + 1);
    }

    public Trigger extendIntake() {
        return new Trigger(() -> false);
    }

    public Trigger retractIntake() {
        return new Trigger(() -> false);
    }

    public Trigger launch() {
        return new Trigger(() -> false);
    }

    public Trigger toggleFeeder() {
        return new Trigger(() -> false);
    }
    
    //Remove later
    public Trigger leftArmUp() {
        return new Trigger(() -> (getRawButton(SpektrumButton.CUp.value)));
    }
    public Trigger leftArmDown() {
        return new Trigger(() -> (getRawButton(SpektrumButton.CDown.value)));
    }
    public Trigger rightArmUp() {
        return new Trigger(() -> (getRawButton(SpektrumButton.FUp.value)));
    }
    public Trigger rightArmDown() {
        return new Trigger(() -> (getRawButton(SpektrumButton.FDown.value)));
    }
}

