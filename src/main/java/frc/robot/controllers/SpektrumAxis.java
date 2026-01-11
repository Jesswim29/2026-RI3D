package frc.robot.controllers;

public enum SpektrumAxis {
    RotateAxis(0),
    Throttle(1),
    LeftSlider(2),
    DriveX(3),
    DriveY(4),
    RightSlider(5),
    UnknownAxis(6),
    TopRightKnob(7);

    public final int value;

    SpektrumAxis(int value) {
        this.value = value;
    }
}
