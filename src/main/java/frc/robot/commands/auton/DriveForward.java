package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.gyros.Gyro;
import frc.robot.subsystems.Drive;

public class DriveForward extends SequentialCommandGroup {

    public DriveForward(Drive drive, Gyro gyro) {
        addCommands(new UniversalDriveCommand(drive, gyro, 0, 60, 0, 0.5));
    }
}
