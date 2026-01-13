package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ClimbUp extends Command {

    boolean down;
    //boolean extend = false;

    public ClimbUp(Climb climber, boolean down) {
        this.down = down;
        this.climber = climber;
    }

    //FOR LATER: Implement climb pattern
    @Override
    public void execute() {
        if(down){
            
        }
        else{

        }
    }
}
