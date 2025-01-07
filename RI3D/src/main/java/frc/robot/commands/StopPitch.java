package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CoralPitcherinator;

public class StopPitch extends InstantCommand{
    private final CoralPitcherinator m_pitcher;
    public StopPitch(CoralPitcherinator pitcher) {
         m_pitcher = pitcher;
         addRequirements(m_pitcher);
    }
    
    @Override
    public void initialize() {
        m_pitcher.setVelocity(0);
    }
}