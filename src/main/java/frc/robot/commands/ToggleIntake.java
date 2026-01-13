package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ToggleIntake extends Command {

    boolean extend = false;
    Intake intake;

    public ToggleIntake(boolean extend, Intake intake) {
        this.extend = extend;
        this.intake = intake;
    }

    @Override
    public void initialize() {
        if (extend) {
            intake.setPivotPos(120);
            intake.setRollerSpeed(.5);
        } else {
            intake.setPivotPos(30);
            intake.setRollerSpeed(0);
        }
    }

    @Override
    public void execute() {
        //here for examples and so andrew can copy paste (im lazy ok)
    }

    @Override
    public void end(boolean interrupted) {
        //empty
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
