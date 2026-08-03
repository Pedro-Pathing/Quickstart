package org.firstinspires.ftc.teamcode.pedroPathing.subSystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.constants.TeleOpConstants.*;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Manages automatic and manual servo movements. */
public class ServoAnimationHandler {

    private final Servo servo;
    private final ElapsedTime animationTimer = new ElapsedTime();
    private boolean isAnimating = false;

    public ServoAnimationHandler(Servo servo) {
        this.servo = servo;
    }

    /** Updates the servo position based on user input or active animation state. */
    public void update(boolean wiggleButton, boolean homeButton) {
        if (wiggleButton) {
            if (((int) (animationTimer.seconds() * (1.0 / ANIMATION_STEP_SEC))) % 2 == 0) {
                servo.setPosition(SERVO_MAX);
            } else {
                servo.setPosition(SERVO_HOME);
            }
            isAnimating = false;
        } else if (homeButton) {
            servo.setPosition(SERVO_HOME);
            isAnimating = false;
        } else if (isAnimating) {
            if (animationTimer.seconds() < ANIMATION_DURATION_SEC) {
                if (((int) (animationTimer.seconds() * (1.0 / ANIMATION_STEP_SEC))) % 2 == 0) {
                    servo.setPosition(SERVO_MAX);
                } else {
                    servo.setPosition(SERVO_HOME);
                }
            } else {
                isAnimating = false;
                servo.setPosition(SERVO_HOME);
            }
        }
    }

    /** Starts the automatic 6-rotation back-and-forth animation. */
    public void startAnimation() {
        isAnimating = true;
        animationTimer.reset();
    }
}
