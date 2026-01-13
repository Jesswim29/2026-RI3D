package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class ToggleShoot extends Command {

    boolean jorbles = true;
    private Launcher launch;

    public ToggleShoot(Launcher Launch, boolean jorbie) {
        this.launch = Launch;
        this.jorbles = jorbie;
    }

    @Override
    public void execute() {
        if (jorbles) {
            launch.setVelocity(0.8);
        }
        //else launch.stopMotors();
    }

    @Override
    public void end(boolean interrupted) {
        launch.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
