package frc.robot.controllers;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SpektrumDriveController
    extends GenericHID
    implements DriveController
{

    // move to constants perchance?
    double driveXFactor = 1.27;
    double driveYFactor = 1.26;
    double throttleFactor = 1.65;
    double rotateFactor = 1;
    double rotationDeadBand = 0.1;
    double xDeadBand = .1;
    double yDeadBand = 0.1;

    public SpektrumDriveController(int port) {
        super(port);
        HAL.report(tResourceType.kResourceType_XboxController, port + 1);
    }

    public double getDriveX() {
        if (getRawAxis(SpektrumAxis.DriveX.value) * driveXFactor > 1) {
            return 1;
        }
        if (getRawAxis(SpektrumAxis.DriveX.value) * driveXFactor < -1) {
            return -1;
        }
        if (
            getRawAxis(SpektrumAxis.DriveX.value) * driveXFactor >
                (xDeadBand * -1) &&
            getRawAxis(SpektrumAxis.DriveX.value) * driveXFactor < xDeadBand
        ) {
            return 0;
        }
        return getRawAxis(SpektrumAxis.DriveX.value) * driveXFactor;
    }

    public double getDriveY() {
        if (getRawAxis(SpektrumAxis.DriveY.value) * driveYFactor > 1) {
            return -1;
        }
        if (getRawAxis(SpektrumAxis.DriveY.value) * driveYFactor < -1) {
            return 1;
        }
        if (
            getRawAxis(SpektrumAxis.DriveY.value) * driveYFactor >
                yDeadBand * -1 &&
            getRawAxis(SpektrumAxis.DriveY.value) * driveYFactor < yDeadBand
        ) {
            return 0;
        }
        return (getRawAxis(SpektrumAxis.DriveY.value) * driveYFactor) * -1;
    }

    public double getThrottle() {
        SmartDashboard.putNumber(
            "Raw",
            getRawAxis(SpektrumAxis.Throttle.value)
        );
        return MathUtil.clamp(
            getRawAxis(SpektrumAxis.Throttle.value) * throttleFactor,
            -1,
            1
        );
    }

    public double getRotation() {
        if (getRawAxis(SpektrumAxis.RotateAxis.value) * rotateFactor > 1) {
            return -1;
        } else if (
            getRawAxis(SpektrumAxis.RotateAxis.value) * rotateFactor < -1
        ) {
            return 1;
        }
        if (
            getRawAxis(SpektrumAxis.RotateAxis.value) * rotateFactor >
                -rotationDeadBand &&
            getRawAxis(SpektrumAxis.RotateAxis.value) * rotateFactor <
            rotationDeadBand
        ) {
            return 0;
        }
        return getRawAxis(SpektrumAxis.RotateAxis.value) * rotateFactor;
    }

    public Trigger reset() {
        return new Trigger(() -> (getRawButton(SpektrumButton.Reset.value)));
    }
}
