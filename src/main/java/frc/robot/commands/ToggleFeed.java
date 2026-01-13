package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.ToLongBiFunction;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class ToggleFeed extends Command{
    boolean jorbles = true;
    private Feed feed;

    public ToggleFeed(Feed feeder, boolean jorbie) {
        this.feed = feeder;
        this.jorbles = jorbie;
    }
    @Override
    public void execute() {
        if(jorbles) {
            feed.Activate();
        }
        else feed.stopMotors();

    }

    
}
