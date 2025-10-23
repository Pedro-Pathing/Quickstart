package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;

public class TurretSubsystem extends RE_SubsystemBase {

    private final Servo turret;
    private final CameraSubsystem camera;

    // PID state
    private double integral = 0.0;
    private double lastErrorDeg = 0.0;
    private long lastNanos = 0L;

  
    private double errFiltDeg = 0.0;

    // avoid the big jumps
    private static final double MIN_DT = 1e-3;  // 1 ms
    private static final double MAX_DT = 0.05;  // 50 ms

    private boolean autoAimEnabled = false;

    public TurretSubsystem(HardwareMap hw, String servoName, CameraSubsystem camera) {
        this.turret = hw.get(Servo.class, servoName);
        this.camera = camera;

        setTurretDeg(0.0);
        Robot.getInstance().subsystems.add(this);
        lastNanos = System.nanoTime();
    }


    public void enableAutoAim(boolean enable) {
        autoAimEnabled = enable;
        resetPID();
    }


    public void setTurretDeg(double deg) {
        double clamped = clamp(deg, Constants.minDeg, Constants.maxDeg);
        double pos = mapDegToPos(clamped);
        turret.setPosition(pos);
    }


    public double getTurretDeg() {
        return mapPosToDeg(turret.getPosition());
    }




    @Override
    public void periodic() {
        if (!autoAimEnabled) return;

        long now = System.nanoTime();
        double dt = (now - lastNanos) / 1e9;
        lastNanos = now;


        if (dt < MIN_DT) dt = MIN_DT;
        if (dt > MAX_DT) dt = MAX_DT;

        // If no target, hold current orientation; don't integrate
        if (!camera.hasBasket()) {
            lastErrorDeg = 0.0;
            return;
        }

        // Positive error means target is to the left; turn left
        double yawErrDeg = camera.getBasketYawDeg();

        // Deadband to stop unnnesary movements
        if (Math.abs(yawErrDeg) < Constants.deadbandDeg) {
            yawErrDeg = 0.0;
        }

        errFiltDeg = Constants.errAlpha * yawErrDeg + (1.0 - Constants.errAlpha) * errFiltDeg;

        // PID

        integral += errFiltDeg * dt;
        if (yawErrDeg == 0.0) {
            integral *= 0.5;
        }
        // stop windup
        integral = clamp(integral, -Constants.maxIntegral, Constants.maxIntegral);


        double deriv = (errFiltDeg - lastErrorDeg) / dt;
        deriv = clamp(deriv, -Constants.maxDeriv, Constants.maxDeriv);
        lastErrorDeg = errFiltDeg;

        // PID output
        double correctionDeg = Constants.kP * errFiltDeg
                + Constants.kI * integral
                + Constants.kD * deriv;

        // Slew-rate limit (deg/s)
        double maxStep = Constants.maxStepDegPerSec * dt;
        correctionDeg = clamp(correctionDeg, -maxStep, maxStep);

        // Apply as an incremental move, then clamp to mechanical limits
        double newDeg = clamp(getTurretDeg() + correctionDeg, Constants.minDeg, Constants.maxDeg);

        // If saturated at a mechanical stop, bleed the integrator to prevent wind-up
        if (newDeg == Constants.minDeg || newDeg == Constants.maxDeg) {
            integral *= 0.9; // slow bleed at end-stops
        }

        setTurretDeg(newDeg);
    }

    // ---------------- Helpers ----------------

    private void resetPID() {
        integral = 0.0;
        lastErrorDeg = 0.0;
        errFiltDeg = 0.0;
        lastNanos = System.nanoTime();
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private double mapDegToPos(double deg) {
        double spanDeg = (Constants.maxDeg - Constants.minDeg);
        double spanPos = (Constants.maxPos - Constants.minPos);
        if (spanDeg == 0.0) return Constants.minPos; // safety
        double t = (deg - Constants.minDeg) / spanDeg;
        return Constants.minPos + t * spanPos;
    }

    private double mapPosToDeg(double pos) {
        double spanDeg = (Constants.maxDeg - Constants.minDeg);
        double spanPos = (Constants.maxPos - Constants.minPos);
        if (spanPos == 0.0) return Constants.minDeg; // safety
        double t = (pos - Constants.minPos) / spanPos;
        return Constants.minDeg + t * spanDeg;
    }
}