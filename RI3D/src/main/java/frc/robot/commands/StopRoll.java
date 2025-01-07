package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.AlgaeRoller;

public class StopRoll extends InstantCommand{
    private final AlgaeRoller m_roller;
    public StopRoll(AlgaeRoller roller) {
         m_roller = roller;
         addRequirements(m_roller);
    }
    
    @Override
    public void initialize() {
        m_roller.setVelocity(0);
    }
}