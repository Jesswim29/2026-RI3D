package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class ClimbUpDown extends Command {

    public static boolean limitSwitchLeft = false;
    public static boolean limitSwitchRight = false;
    boolean down;
    Climb climber;
    boolean leftArm;

    public ClimbUpDown(Climb climber, boolean down, boolean leftArm) {
        this.down = down;
        this.climber = climber;
        this.leftArm = leftArm;
    }

    @Override
    public void execute() {
        if (leftArm) {
            if (down && limitSwitchLeft == false) {
                climber.retractClimber(Constants.ClimberParams.leftID);
            } else {
                climber.extendClimber(Constants.ClimberParams.leftID);
            }
        } else {
            if (down && limitSwitchRight == false) {
                climber.retractClimber(Constants.ClimberParams.rightID);
            } else {
                climber.extendClimber(Constants.ClimberParams.rightID);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // if(leftArm){
        SmartDashboard.putString("THE POSITION: ", "1rd");
        climber.stopClimber(Constants.ClimberParams.rightID);
        // }
        // else{
        // SmartDashboard.putString("THE POSITION: ", "2rd");
        climber.stopClimber(Constants.ClimberParams.leftID);
        // }
        // SmartDashboard.putString("THE POSITION: ", "3rd");
    }
}
