package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.REVMagneticLimit;

public class TestLimit extends SubsystemBase {

    REVMagneticLimit limitA;
    REVMagneticLimit limitB;

    public TestLimit() {
        limitA = new REVMagneticLimit(0);
        limitB = new REVMagneticLimit(1);
    }

    @Override
    public void periodic() {
        frc.robot.commands.ClimbUpDown.limitSwitchLeft = limitA
            .get()
            .getAsBoolean();
        frc.robot.commands.ClimbUpDown.limitSwitchRight = limitB
            .get()
            .getAsBoolean();

        SmartDashboard.putBoolean(
            "left limit",
            frc.robot.commands.ClimbUpDown.limitSwitchLeft
        );

        SmartDashboard.putBoolean(
            "right limit",
            frc.robot.commands.ClimbUpDown.limitSwitchRight
        );
    }
}
