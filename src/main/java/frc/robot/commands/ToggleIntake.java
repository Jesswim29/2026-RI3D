package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class ToggleIntake extends Command {

    boolean extend = true;

    public ToggleIntake(boolean extend) {
        this.extend = extend;
    }

    @Override
    public void execute() {
        if (extend) {
            // move out
        } else {
            // move in
        }
    }
}
