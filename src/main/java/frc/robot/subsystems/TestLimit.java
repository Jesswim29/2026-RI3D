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
        SmartDashboard.putBoolean("Limit A", limitA.get().getAsBoolean());
        SmartDashboard.putBoolean("Limit B", limitB.get().getAsBoolean());
    }
}
