// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.ToggleIntake;
import frc.robot.commands.auton.BasicAuto;
import frc.robot.controllers.DriveController;
import frc.robot.controllers.OperatorController;
import frc.robot.controllers.SpektrumDriveController;
import frc.robot.controllers.XboxOperatorController;
import frc.robot.gyros.Gyro;
import frc.robot.gyros.NavxGyro;
import frc.robot.subsystems.Drive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems and commands are defined here...
    private final Gyro gyro = new NavxGyro();
    //new XboxDriveController(new XboxController(Constants.kDriveController)),
    private final DriveController driveController = new SpektrumDriveController(
        Constants.kDriveController
    );

    private final OperatorController operatorController =
        new XboxOperatorController(
            new XboxController(Constants.kOperatorController)
        );
    private final Drive drive = new Drive(gyro);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        drive.setDefaultCommand(new TeleopDrive(driveController, drive, gyro));

        // Button mappings
        bindButtons();

        new frc.robot.subsystems.TestLimit();
    }

    private void bindButtons() {
        // Drive Controls
        driveController.reset().onTrue(new ResetGyro(gyro));

        // Operator Controls
        operatorController.extendIntake().onTrue(new ToggleIntake(true));
        operatorController.retractIntake().onTrue(new ToggleIntake(false));
    }

    public Command getAutonomousCommand() {
        return new BasicAuto(drive, gyro);
    }
}
