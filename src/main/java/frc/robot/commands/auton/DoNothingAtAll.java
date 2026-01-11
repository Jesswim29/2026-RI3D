package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;

public class DoNothingAtAll extends Command {

    public DoNothingAtAll() {}

    @Override
    public void initialize() {}

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
