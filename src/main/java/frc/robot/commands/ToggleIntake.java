package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class ToggleIntake extends Command {

    boolean extend = false;
    Intake intake;
    double speed = 0;

    double point;

    private PIDController pivotPID;

    public ToggleIntake(boolean extend, Intake intake) {
        this.extend = extend;
        this.intake = intake;

        pivotPID = new PIDController(
            Constants.IntakeConstants.kP,
            Constants.IntakeConstants.kI,
            Constants.IntakeConstants.kD
        );
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        // if (extend) {
        pivotPID.setSetpoint(110);
        // point = 110;
        // intake.setPivotPos(120);
        intake.setRollerSpeed(.35); //.5

        // speed = 1 - intake.getAbsolutePivotPos()/point * .5;
        // if(point > intake.getAbsolutePivotPos()){

        // }else{

        // }
        // } else {
        //     pivotPID.setSetpoint(30);
        //     // intake.setPivotPos(30);
        //     intake.setRollerSpeed(0);
        // }
        speed = pivotPID.calculate(intake.getAbsolutePivotPos());

        intake.setPivotSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
