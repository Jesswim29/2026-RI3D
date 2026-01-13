package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.gyros.Gyro;

public class ResetGyro extends Command {

    private Gyro gyro;

    public ResetGyro(Gyro gyro) {
        this.gyro = gyro;
    }

    @Override
    public void initialize() {
        System.out.println("Reset gyro...");
        gyro.zero();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
