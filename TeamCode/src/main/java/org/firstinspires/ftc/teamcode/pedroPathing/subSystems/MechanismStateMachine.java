package org.firstinspires.ftc.teamcode.pedroPathing.subSystems;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.slideConstants;
import com.qualcomm.robotcore.hardware.Gamepad;

/** Handles the robot's high-level mechanism states for non-blocking automation. */
public class MechanismStateMachine {

    public enum State {
        IDLE,
        LIFTING_TO_HIGH,
        LIFTING_TO_MIDDLE,
        LOWERING,
        RESETTING
    }

    private State currentState = State.IDLE;
    private final slideConstants slide;
    private final ServoAnimationHandler servoAnim;

    public MechanismStateMachine(slideConstants slide, ServoAnimationHandler servoAnim) {
        this.slide = slide;
        this.servoAnim = servoAnim;
    }

    /** Updates the active mechanism state based on driver input and sensor data. */
    public void update(Gamepad gamepad) {
        if (gamepad.dpad_up) {
            currentState = State.LIFTING_TO_HIGH;
            slide.extendToHigh();
            servoAnim.startAnimation();
        } else if (gamepad.right_bumper) {
            currentState = State.LIFTING_TO_MIDDLE;
            slide.extendToMiddle();
        } else if (gamepad.dpad_down) {
            currentState = State.LOWERING;
            slide.extendToBottom();
        } else if (gamepad.back) {
            currentState = State.RESETTING;
            slide.resetEncoder();
        }

        servoAnim.update(gamepad.dpad_right, gamepad.dpad_left);
        slide.update(); // Added gravity feedforward update
    }

    /** Returns the current active high-level state of the mechanisms. */
    public State getState() { return currentState; }
}
