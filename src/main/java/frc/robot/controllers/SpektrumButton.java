package frc.robot.controllers;

public enum SpektrumButton {
    A(1),
    BBackward(2),
    BForward(3),
    CUp(4),
    CDown(5),
    DUp(6),
    DDown(7),
    FDown(8),
    FUp(9),
    GDown(10),
    GUp(11),
    H(12),
    I(13),
    Reset(14),
    Cancel(15),
    SelectPress(16),
    SelectLeft(17),
    SelectRight(18),
    BelowLeftStickLeft(19),
    BelowLeftStickRight(20),
    BesidesLeftStickDown(21),
    BesidesLeftStickUp(22),
    BelowRightStickLeft(23),
    BelowRightStickRight(24),
    BesidesRightStickDown(25),
    BesidesRightStickUp(26),
    UnknownButton(27); // TODO: WHYYYYY?

    public final int value;

    SpektrumButton(int value) {
        this.value = value;
    }
}
