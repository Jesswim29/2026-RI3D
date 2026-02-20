// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* DO NOT TOUCH
 * Example template for creating new commands for subsystems
 * Probably in the teleop period?
 */
package frc.robot.commands;

import frc.lib.climbState;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class UniversalClimb extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Climb climb;
  climbState climbState;

  public UniversalClimb(Climb climb, climbState climbState) {
    this.climb = climb;
    this.climbState = climbState;
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        //TODO add limit switches
        switch (climbState){
            case Up:
                climb.up();
            case Down:
                climb.down();
            case Stop:
                climb.stop();
        }  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
