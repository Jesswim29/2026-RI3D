// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Gyro;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopDrive extends Command {

    /** Creates a new TeleopDrive. */
    private DoubleSupplier translation;
    private DoubleSupplier strafe;
    private DoubleSupplier rotation;
    // The following is seemingly not being used anywhere
    // TODO: Reimplement gyro zeroing (please for the love of something give this variable a sane name, not gyroResetButton)
    private BooleanSupplier gyroResetButton;

    private Drive m_drive;
    private Gyro m_gyro;

    public TeleopDrive(
        DoubleSupplier translationBut,
        DoubleSupplier strafeBut,
        DoubleSupplier rotation,
        BooleanSupplier a,
        Drive drive,
        Gyro gyro
    ) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.translation = translationBut;
        this.strafe = strafeBut;
        this.rotation = rotation;
        this.gyroResetButton = a;

        m_drive = drive;
        m_gyro = gyro;
        addRequirements(m_drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double yVal = MathUtil.applyDeadband(
            translation.getAsDouble(),
            Constants.kDeadzone
        );

        double xVal = MathUtil.applyDeadband(
            strafe.getAsDouble(),
            Constants.kDeadzone
        );

        double rotVal = MathUtil.applyDeadband(
            rotation.getAsDouble(),
            Constants.kDeadzone
        );

        // commented: see above
        //if (gyroResetButton.getAsBoolean()) {
        //    m_gyro.zeroGyro();
        //}

        m_drive.swerve(
            new Translation2d(-yVal, xVal).times(DrivetrainConstants.maxSpeed),
            rotVal * DrivetrainConstants.maxTurningSpeed,
            false
        ); // TODO: Get this to work
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
