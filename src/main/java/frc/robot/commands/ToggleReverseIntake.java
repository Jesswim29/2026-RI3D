package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ToggleReverseIntake extends Command {

    private Intake intake;
    private boolean value;

    public ToggleReverseIntake(Intake intake, boolean value) {
        this.intake = intake;
        this.value = value;
    }

    @Override
    public void initialize() {
        intake.reverse = value;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
