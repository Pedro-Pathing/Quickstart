package org.firstinspires.ftc.teamcode.pedroPathing.subSystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.constants.TeleOpConstants.*;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/** Manages the "Settled Lock" rotation logic for stable field-centric driving. */
public class HeadingLockHandler {

    private final Follower follower;
    private final ElapsedTime timer = new ElapsedTime();
    
    private double targetHeading = 0.0;
    private double lastError = 0.0;
    private double lastHeading = 0.0;
    private boolean isCoasting = false;

    public HeadingLockHandler(Follower follower) {
        this.follower = follower;
        timer.reset();
    }

    /** Calculates the rotation power needed to maintain a locked heading or allow manual turning. */
    public double calculateRotationPower(double rawRx, double translationMagnitude) {
        double currentHeading = follower.getPose().getHeading();
        double dt = timer.seconds();
        timer.reset();

        double angularVelocity = (dt > 0) ? (currentHeading - lastHeading) / dt : 0.0;
        lastHeading = currentHeading;

        double rx = 0.0;
        double absRx = Math.abs(rawRx);

        if (absRx > DEADZONE) {
            double normalizedRx = (absRx - DEADZONE) / (1.0 - DEADZONE);
            rx = Math.signum(rawRx) * Math.pow(normalizedRx, 3) * TURN_SPEED_LIMIT;
            isCoasting = true; 
            targetHeading = currentHeading;
            lastError = 0.0;
        } else {
            if (isCoasting && Math.abs(angularVelocity) < 0.1) {
                isCoasting = false;
                targetHeading = currentHeading;
            }

            if (isCoasting) {
                rx = 0.0;
                targetHeading = currentHeading;
                lastError = 0.0;
            } else {
                double headingError = targetHeading - currentHeading;
                headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));

                if (Math.abs(headingError) < HEADING_LOCK_TOLERANCE_RAD) {
                    rx = 0.0;
                } else {
                    double derivative = (dt > 0) ? (headingError - lastError) / dt : 0.0;
                    rx = (headingError * HEADING_LOCK_KP) + (derivative * HEADING_LOCK_KD);
                    rx = Range.clip(rx, -TURN_SPEED_LIMIT, TURN_SPEED_LIMIT);
                }
                lastError = headingError;
            }
        }
        return rx;
    }

    /** Synchronizes the internal target heading to a new absolute angle. */
    public void resetHeading(double newHeading) {
        targetHeading = newHeading;
        lastHeading = newHeading;
        lastError = 0.0;
        isCoasting = false;
        timer.reset(); // Added to prevent PID power spikes when resuming manual control
    }

    /** Returns the current active target heading for the lock. */
    public double getTargetHeading() { return targetHeading; }
}
