// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ElevateDown;
import frc.robot.commands.ElevateUp;
import frc.robot.commands.Swerve.TeleopDrive;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Gyro m_gyro = new Gyro();
  private final Drive m_drive = new Drive(m_gyro);
  private final Elevator m_elevator = new Elevator();

  private final XboxController m_driver = new XboxController(Constants.kDriveController);
  private final CommandXboxController m_controller = new CommandXboxController(Constants.kButtonController);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();



    m_controller.a().onTrue(new InstantCommand(() -> new ElevateDown(m_elevator)) {});
    m_controller.b().onTrue(new InstantCommand(() -> new ElevateUp(m_elevator)) {});
    // m_drive.setDefaultCommand(
    //   new TeleopDrive(
    //     () -> m_driver.getLeftY(),
    //     () -> -m_driver.getLeftX(),
    //     () -> m_driver.getRightX(),
    //     () -> m_driver.getAButton(),
    //     m_drive
    //   )
    // );
    
    m_controller.x().onTrue(new InstantCommand() {
      @Override
      public void initialize() {
  
          m_drive.ZeroWheels();
          System.out.println("Zeroing the wheels");
        
      };
    }); 
    m_controller.y().onTrue(new InstantCommand(){
      @Override
      public void initialize() {
        m_gyro.zeroGyro();

      };
    });

    
    m_controller.start().onTrue(new InstantCommand() {
      @Override
      public void initialize() {
          System.out.println("moving to " + m_drive.getAngle());
      }
    });
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new WaitCommand(15);
  }
}
