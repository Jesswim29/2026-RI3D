// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autons;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autons.BaseAutonCommands.UniversalDriveCommand;
import frc.robot.gyros.Gyro;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BasicAuto extends SequentialCommandGroup {

  public BasicAuto(Drive drive, Gyro gyro) {
    addCommands(
      new UniversalDriveCommand(drive, gyro, 180, 100, 0, 0.5),
      new UniversalDriveCommand(drive, gyro, 37, 65, 180, 0.5)
    );
  }
}
