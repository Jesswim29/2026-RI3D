package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class REVMagneticLimit {

    private DigitalInput input;
    private Trigger trigger;

    public REVMagneticLimit(int id) {
        this.input = new DigitalInput(id);
        this.trigger = new Trigger(() -> (input.get() == false));
    }

    public Trigger get() {
        return trigger;
    }
}
