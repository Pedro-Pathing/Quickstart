package org.firstinspires.ftc.teamcode.hardwareControl.sensors.gamepad;

import org.firstinspires.ftc.teamcode.hardwareConfig.sensors.gamepad.GamepadSnapshot;

public final class GamepadDelta {

    /* ===================== Buttons ===================== */

    // Face buttons
    public final boolean aPressed, aReleased;
    public final boolean bPressed, bReleased;
    public final boolean xPressed, xReleased;
    public final boolean yPressed, yReleased;

    // D-pad
    public final boolean dpadUpPressed, dpadUpReleased;
    public final boolean dpadDownPressed, dpadDownReleased;
    public final boolean dpadLeftPressed, dpadLeftReleased;
    public final boolean dpadRightPressed, dpadRightReleased;

    // Bumpers
    public final boolean leftBumperPressed, leftBumperReleased;
    public final boolean rightBumperPressed, rightBumperReleased;

    /* ===================== Triggers ===================== */

    public final float leftTriggerDelta;
    public final float rightTriggerDelta;

    public final boolean leftTriggerPulling;
    public final boolean leftTriggerReleasing;
    public final boolean rightTriggerPulling;
    public final boolean rightTriggerReleasing;

    /* ===================== Sticks ===================== */

    public final float leftStickXDelta;
    public final float leftStickYDelta;
    public final float rightStickXDelta;
    public final float rightStickYDelta;

    private static final float TRIGGER_THRESHOLD = 0.5f;

    public GamepadDelta(GamepadSnapshot prev, GamepadSnapshot curr) {

        /* ---------- Face buttons ---------- */
        aPressed = curr.a && !prev.a;
        aReleased = !curr.a && prev.a;

        bPressed = curr.b && !prev.b;
        bReleased = !curr.b && prev.b;

        xPressed = curr.x && !prev.x;
        xReleased = !curr.x && prev.x;

        yPressed = curr.y && !prev.y;
        yReleased = !curr.y && prev.y;

        /* ---------- D-pad ---------- */
        dpadUpPressed = curr.dpadUp && !prev.dpadUp;
        dpadUpReleased = !curr.dpadUp && prev.dpadUp;

        dpadDownPressed = curr.dpadDown && !prev.dpadDown;
        dpadDownReleased = !curr.dpadDown && prev.dpadDown;

        dpadLeftPressed = curr.dpadLeft && !prev.dpadLeft;
        dpadLeftReleased = !curr.dpadLeft && prev.dpadLeft;

        dpadRightPressed = curr.dpadRight && !prev.dpadRight;
        dpadRightReleased = !curr.dpadRight && prev.dpadRight;

        /* ---------- Bumpers ---------- */
        leftBumperPressed = curr.leftBumper && !prev.leftBumper;
        leftBumperReleased = !curr.leftBumper && prev.leftBumper;

        rightBumperPressed = curr.rightBumper && !prev.rightBumper;
        rightBumperReleased = !curr.rightBumper && prev.rightBumper;

        /* ---------- Triggers ---------- */
        leftTriggerDelta = curr.leftTrigger - prev.leftTrigger;
        rightTriggerDelta = curr.rightTrigger - prev.rightTrigger;

        leftTriggerPulling =
                prev.leftTrigger < TRIGGER_THRESHOLD &&
                        curr.leftTrigger >= TRIGGER_THRESHOLD;

        leftTriggerReleasing =
                prev.leftTrigger >= TRIGGER_THRESHOLD &&
                        curr.leftTrigger < TRIGGER_THRESHOLD;

        rightTriggerPulling =
                prev.rightTrigger < TRIGGER_THRESHOLD &&
                        curr.rightTrigger >= TRIGGER_THRESHOLD;

        rightTriggerReleasing =
                prev.rightTrigger >= TRIGGER_THRESHOLD &&
                        curr.rightTrigger < TRIGGER_THRESHOLD;

        /* ---------- Sticks ---------- */
        leftStickXDelta = curr.leftStickX - prev.leftStickX;
        leftStickYDelta = curr.leftStickY - prev.leftStickY;
        rightStickXDelta = curr.rightStickX - prev.rightStickX;
        rightStickYDelta = curr.rightStickY - prev.rightStickY;
    }
}
