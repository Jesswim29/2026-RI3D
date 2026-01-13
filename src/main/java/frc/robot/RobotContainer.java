// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ClimbUpDown;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.ToggleFeed;
import frc.robot.commands.ToggleIntake;
import frc.robot.commands.ToggleShoot;
import frc.robot.commands.auton.BasicAuto;
import frc.robot.controllers.DriveController;
import frc.robot.controllers.OperatorController;
import frc.robot.controllers.SpektrumDriveController;
import frc.robot.controllers.XboxOperatorController;
import frc.robot.gyros.Gyro;
import frc.robot.gyros.NavxGyro;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems and commands are defined here...
    private final Gyro gyro = new NavxGyro();
    private final Launcher launcher = new Launcher();
    private final Feed feed = new Feed();
    //new XboxDriveController(new XboxController(Constants.kDriveController)),
    private final DriveController driveController = new SpektrumDriveController(
        Constants.kDriveController
    );
    private final Climb climb = new Climb();

    private final OperatorController operatorController =
        new XboxOperatorController(
            new XboxController(Constants.kOperatorController)
        );
    private final Drive drive = new Drive(gyro);

    private final Intake intake = new Intake();

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
        driveController
            .leftArmUp()
            .whileTrue(new ClimbUpDown(climb, false, true));
        driveController
            .leftArmDown()
            .whileTrue(new ClimbUpDown(climb, true, true));
        driveController
            .rightArmUp()
            .whileTrue(new ClimbUpDown(climb, false, false));
        driveController
            .rightArmDown()
            .whileTrue(new ClimbUpDown(climb, true, false));

        // Operator Controls
        operatorController
            .extendIntake()
            .whileTrue(new ToggleIntake(true, intake));
        // operatorController.retractIntake().whileTrue(new ToggleIntake(false, intake));
        operatorController
            .retractIntake()
            .whileTrue(new IntakeIn(false, intake));

        // Launcher subcommands
        operatorController.toggleFeeder().whileTrue(new ToggleFeed(feed, true));
        operatorController.launch().whileTrue(new ToggleShoot(launcher, true));
    }

    public Command getAutonomousCommand() {
        return new BasicAuto(drive, gyro);
    }
}
