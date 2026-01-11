package frc.robot.commands.auton;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.gyros.Gyro;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Wheel;

public class UniversalDriveCommand extends Command {

    protected Drive drive;
    protected Gyro gyro;
    protected Wheel[] _wheel;

    protected double encoderPosAvg;
    protected double percentDone = 0;
    protected double driveAngle;
    protected double finalDriveAngle;
    protected double rotationAngle;
    protected double wantedDistance = 0;
    protected double travelledDistance = 0;
    protected double startDist = 0;
    protected double speed;
    protected double calculateOutput;
    protected PIDController pidController;
    protected SlewRateLimiter slewRateLimiter;
    protected double trapezoidalSpeed = 0;
    protected double stopPercent = 0;
    double rotateDist = 0;
    boolean atAngle = false;

    /**
     * The best drive command.
     * Field oriented driving with speed modifier and abillity to face desired angle
     *
     * @param drive The drive
     * @param gyro The gyro
     * @param driveAngle Direction to drive in degrees (field oriented)
     * @param wantedDistance Distance to drive in inches
     * @param rotationAngle Direction to point the robot to in degrees
     * @param speed Flat multiplier to the speed of the command
     */
    public UniversalDriveCommand(
        Drive drive,
        Gyro gyro,
        double driveAngle,
        double wantedDistance,
        double rotationAngle,
        double speed
    ) {
        drive = drive;
        gyro = gyro;
        travelledDistance = 0;
        percentDone = 0;
        this.driveAngle = driveAngle;
        this.rotationAngle = rotationAngle;
        this.wantedDistance = wantedDistance;
        this.speed = speed;
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        slewRateLimiter = new SlewRateLimiter(2.5); //2.5
        //reset percent done and setup PID
        percentDone = 0;
        pidController = new PIDController(
            Constants.DriveCommandConstants.P,
            Constants.DriveCommandConstants.I,
            Constants.DriveCommandConstants.D
        );
        pidController.setTolerance(1);

        //find our start distance
        double startTick = 0;
        Wheel[] wheels = drive.getWheels();
        for (int i = 0; i < wheels.length; i++) {
            startTick += Math.abs(drive.getWheels()[i].getDistance());
            drive.getWheels()[i].resetEncoders();
            drive.getWheels()[i].resetEncoders();
            drive.getWheels()[i].resetEncoders();
            drive.getWheels()[i].resetEncoders();
            drive.getWheels()[i].resetEncoders();
            // startDist += Math.abs(drive.getWheels()[i].getEncoderDistance());
        }
        startTick = startTick / drive.getWheels().length;
        // startDist = startDist / drive.getWheels().length;
        System.out.println("Start dist: " + startTick);

        //add our rotation distance to the total wanted distance
        SmartDashboard.putNumber("wantedDistance", wantedDistance);
        rotateDist = rotateAccomidation(wheels);
        wantedDistance += rotateDist;
        SmartDashboard.putNumber("wantedDistance Rotate:", wantedDistance);
        // wantedDistance += startDist;
        // SmartDashboard.putNumber("wantedDistance Final", wantedDistance);

        SmartDashboard.putNumber("where to go rotate", driveAngle);
        SmartDashboard.putNumber(
            "where to go rotate the sequel",
            rotationAngle
        );

        //send our wanted distance to the PID
        pidController.setSetpoint(wantedDistance);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //tell the robot what angle to drive at (subtract real gyro angle for field oriented driving)
        finalDriveAngle = driveAngle - gyro.getRealGyroAngle();

        //Get all wheels encoders and average them to find our distance
        encoderPosAvg = 0;
        for (int i = 0; i < drive.getWheels().length; i++) {
            encoderPosAvg += Math.abs(drive.getWheels()[i].getDistance());
        }
        encoderPosAvg = encoderPosAvg / drive.getWheels().length;
        // encoderPosAvg =- startDist;

        //TODO trapezoid + slew rate limit???
        //calculate our PID output based on the encoder average
        // calculateOutput = pidController.calculate(encoderPosAvg);
        double rampedOutput = trapezoidCalculation();
        rampedOutput = rampedOutput * speed;
        SmartDashboard.putNumber(
            "THE CHAINS THE CHAINS THE CHAINS",
            rampedOutput
        );

        //calculate rotation speed based on angle difference
        double rotationSpeed;
        // rotationSpeed = (rotationAngle - gyro.getRealGyroAngle()) * .0025;
        rotationSpeed =
            (rotationAngle -
                ((gyro.getRawGyroAngle() + gyro.getGyroOffset()) % 360)) *
            .0025;
        if (rotationSpeed > .20) {
            rotationSpeed = .20;
        } else if (rotationSpeed < -.20) {
            rotationSpeed = -.20;
        }
        SmartDashboard.putNumber(
            "the incredible speed with which we must rotate the banannananana",
            rotationSpeed
        );
        SmartDashboard.putNumber("encoderPosAvg", encoderPosAvg);

        //send our linear angle, linear speed, and rotation speed to drive
        drive.swerve(finalDriveAngle, rampedOutput, rotationSpeed);

        atAngle =
            gyro.getRealGyroAngle() > (rotationAngle - 1) &&
            gyro.getRealGyroAngle() < (rotationAngle + 1);

        //calculate percent done
        // percentDone = Math.abs(Math.abs(encoderPosAvg - startDist) / wantedDistance);
        percentDone = encoderPosAvg / wantedDistance;
        SmartDashboard.putNumber("Percentage done", percentDone);
    }

    protected double stopDist = 75;

    public double trapezoidCalculation() {
        if (wantedDistance > stopDist + rotateDist) {
            stopPercent = (wantedDistance - stopDist) / wantedDistance;
        } else {
            stopPercent = .30;
        }
        SmartDashboard.putNumber("stopping spot", stopPercent);
        double finalSpeed = 0;
        percentDone = encoderPosAvg / wantedDistance;
        if (percentDone < stopPercent) {
            //.7
            trapezoidalSpeed = 1;
        } else if (percentDone > stopPercent) {
            //.7
            trapezoidalSpeed = .1;
        }
        finalSpeed = slewRateLimiter.calculate(trapezoidalSpeed);
        return finalSpeed;
    }

    public double rotateAccomidation(Wheel[] wheels) {
        //find the angle to rotate to using best angle and the current angle using gyro
        SmartDashboard.putNumber("best angle?", rotationAngle);
        // double currentAng = gyro.getRealGyroAngle();
        double currentAng = ((gyro.getRawGyroAngle() + gyro.getGyroOffset()) %
            360);

        //math to calculate the circumference of the circle drawn by the wheels of the bot when rotating.
        double xPos = Units.metersToInches(wheels[2].get_xPos());
        double yPos = Units.metersToInches(wheels[2].get_yPos());
        double radius = Math.sqrt(Math.pow(xPos, 2) + Math.pow(yPos, 2));
        double circumference = Math.PI * (radius * 2);
        SmartDashboard.putNumber("Circ", circumference);

        //calculate how far the wheels travel based off the percent of 360 we travel times circumference
        SmartDashboard.putNumber("travel", rotationAngle - currentAng);
        double rotationDistance =
            circumference * ((rotationAngle - currentAng) / 360);
        SmartDashboard.putNumber("rotate distance accom", rotationDistance);
        return Math.abs(rotationDistance);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putNumber(
            "FINAL POSITION OF THE COMMAND ",
            encoderPosAvg
        );
        System.out.println("FINAL POSITION OF THE COMMAND: " + encoderPosAvg);
        drive.swerve(0, 0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return percentDone >= 1;
    }
}
