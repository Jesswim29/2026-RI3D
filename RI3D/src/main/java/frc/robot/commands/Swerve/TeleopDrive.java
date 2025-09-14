// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Gyro;

public class TeleopDrive extends Command {
  /** Creates a new TeleopDrive. */
  private DoubleSupplier m_y; // Used to store the Translation value.
  private DoubleSupplier m_x; // Used to store the Strafe value.
  private DoubleSupplier m_rot; // Used to store the Rotation value.
  private BooleanSupplier m_a; // used to store the gyro reset button
  private Drive m_drive;
  private Gyro m_gyro;

  public TeleopDrive(DoubleSupplier y, DoubleSupplier x, DoubleSupplier rot, BooleanSupplier a, Drive drive, Gyro gyro) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_y = y;
    m_x = x;
    m_rot = rot;
    m_a = a;
    m_drive = drive;
    m_gyro = gyro;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var yVal = MathUtil.applyDeadband(m_y.getAsDouble(), Constants.kDeadzone);
    var xVal = MathUtil.applyDeadband(m_x.getAsDouble(), Constants.kDeadzone);
    var rotVal = MathUtil.applyDeadband(m_rot.getAsDouble(), Constants.kDeadzone);

    if (m_a.getAsBoolean()) {
      m_gyro.zeroGyro();
    }

    m_drive.swerve(new Translation2d(-yVal, xVal).times(DrivetrainConstants.maxSpeed),
        rotVal * DrivetrainConstants.maxTurningSpeed, false); // TODO: Get this to work
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
