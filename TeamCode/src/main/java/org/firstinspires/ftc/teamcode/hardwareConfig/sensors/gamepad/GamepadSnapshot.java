package org.firstinspires.ftc.teamcode.hardwareConfig.sensors.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

public final class GamepadSnapshot {

    // Sticks
    public final float leftStickX;
    public final float leftStickY;
    public final float rightStickX;
    public final float rightStickY;

    // Triggers
    public final float leftTrigger;
    public final float rightTrigger;

    // Buttons
    public final boolean a;
    public final boolean b;
    public final boolean x;
    public final boolean y;

    public final boolean dpadUp;
    public final boolean dpadDown;
    public final boolean dpadLeft;
    public final boolean dpadRight;

    public final boolean leftBumper;
    public final boolean rightBumper;

    public final boolean leftStickButton;
    public final boolean rightStickButton;

    public final boolean back;
    public final boolean start;
    public final boolean guide;

    public GamepadSnapshot(Gamepad gp) {
        leftStickX = gp.left_stick_x;
        leftStickY = gp.left_stick_y;
        rightStickX = gp.right_stick_x;
        rightStickY = gp.right_stick_y;

        leftTrigger = gp.left_trigger;
        rightTrigger = gp.right_trigger;

        a = gp.a;
        b = gp.b;
        x = gp.x;
        y = gp.y;

        dpadUp = gp.dpad_up;
        dpadDown = gp.dpad_down;
        dpadLeft = gp.dpad_left;
        dpadRight = gp.dpad_right;

        leftBumper = gp.left_bumper;
        rightBumper = gp.right_bumper;

        leftStickButton = gp.left_stick_button;
        rightStickButton = gp.right_stick_button;

        back = gp.back;
        start = gp.start;
        guide = gp.guide;
    }
}
