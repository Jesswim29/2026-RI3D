// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.controllers.DriveController;
import frc.robot.gyros.Gyro;
import frc.robot.subsystems.Drive;

public class TeleopDrive extends Command {

    /** Creates a new TeleopDrive. */
    private DriveController controller;

    private Drive drive;
    private Gyro gyro;

    private double lastRightDirectional = 0;
    private SlewRateLimiter rampRate;

    /**
     * Teleop drive constructor
     * @param controller Controller object Xbox/Spektrum
     * @param drive
     * @param gyro
     */
    public TeleopDrive(DriveController controller, Drive drive, Gyro gyro) {
        this.controller = controller;
        this.drive = drive;
        this.gyro = gyro;
        this.rampRate = new SlewRateLimiter(1);
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var xCon = controller.getDriveX();
        SmartDashboard.putNumber("xCon", xCon);
        double stickX = MathUtil.applyDeadband(xCon, Constants.kDeadzone);

        var yCon = controller.getDriveY();
        SmartDashboard.putNumber("yCon", yCon);
        double stickY = MathUtil.applyDeadband(yCon, Constants.kDeadzone);
        var rotationCon = controller.getRotation();
        SmartDashboard.putNumber("rotationCon", rotationCon);
        double rotation = MathUtil.applyDeadband(
            controller.getRotation(),
            Constants.kDeadzone
        );

        var throttleCon = controller.getThrottle();
        SmartDashboard.putNumber("throttleCon", throttleCon);
        double throttle = MathUtil.applyDeadband(
            controller.getThrottle(),
            Constants.kDeadzone
        );

        Translation2d rightCoordinate = new Translation2d(stickX, stickY);
        double rightMagnitude =
            (coordinateToMagnitude(rightCoordinate) + 360) % 360;
        double rightDirectional =
            (coordinateToAngle(rightCoordinate) + 360) % 360;

        //Translation2d leftCoordinate = new Translation2d(rotation, throttle);
        //double leftDirectional = coordinateToAngle(leftCoordinate);
        double throttleValue = ((throttle + 1) / 2) + .1;

        // make sure gyro isn't stupid
        rightDirectional = rightDirectional - gyro.getRealGyroAngle();

        double finalLinearMagnitude = rightMagnitude * throttleValue;
        if (finalLinearMagnitude > 1) {
            finalLinearMagnitude = 1;
        } else if (finalLinearMagnitude < -1) {
            finalLinearMagnitude = -1;
        }

        double rampedMagnitude = rampRate.calculate(finalLinearMagnitude);
        double finalRotationMagnitude = rotation * throttleValue;
        if (finalRotationMagnitude > 1) {
            finalRotationMagnitude = 1;
        } else if (finalRotationMagnitude < -1) {
            finalRotationMagnitude = -1;
        }

        SmartDashboard.putNumber("right dir", rightDirectional);
        SmartDashboard.putNumber("ramp mag", rampedMagnitude);
        SmartDashboard.putNumber("final rot", finalRotationMagnitude);

        if (finalLinearMagnitude != 0 || finalRotationMagnitude != 0) {
            drive.swerve(
                rightDirectional,
                rampedMagnitude,
                finalRotationMagnitude
            );
            lastRightDirectional = rightDirectional;
        } else {
            drive.swerve(lastRightDirectional, 0, 0);
        }
        // drive.swerve(0, 0, 0);
    }

    /**
     * Converts X and Y coordinates to angle
     * @param coordinate Translation2d object containing X and Y value
     * @return angle in DEGREES
     */
    public double coordinateToAngle(Translation2d coordinate) {
        return Math.toDegrees(Math.atan2(coordinate.getX(), coordinate.getY()));
    }

    /**
     * Converts X and Y coordinates to length
     * @param coordinate Translation2d object containing X and Y value
     * @return Sum number units pending????
     */
    public double coordinateToMagnitude(Translation2d coordinate) {
        return Math.sqrt(
            Math.pow(coordinate.getX(), 2) + Math.pow(coordinate.getY(), 2)
        );
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
