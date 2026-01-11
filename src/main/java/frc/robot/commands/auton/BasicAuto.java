package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.gyros.Gyro;
import frc.robot.subsystems.Drive;

public class BasicAuto extends SequentialCommandGroup {

    public BasicAuto(Drive drive, Gyro gyro) {
        addCommands(
            new UniversalDriveCommand(drive, gyro, 180, 100, 0, 0.5),
            new UniversalDriveCommand(drive, gyro, 37, 65, 180, 0.5)
        );
    }
}
