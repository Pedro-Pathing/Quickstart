package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;

public class TurretSubsystem extends RE_SubsystemBase {

    private final CRServo turret;
    private final CameraSubsystem camera;

    // PID state (on yaw error)
    private double integral = 0.0;
    private double lastErrDeg = 0.0;
    private double errFiltDeg = 0.0;
    private long lastNanos = 0L;

    // dt clamp to avoid numerical spikes
    private static final double MIN_DT = 1e-3;  // 1 ms
    private static final double MAX_DT = 0.05;  // 50 ms

    private boolean autoAimEnabled = false;

    public TurretSubsystem(HardwareMap hw, String crServoName, CameraSubsystem camera) {
        this.turret = hw.get(CRServo.class, crServoName);
        this.camera = camera;


        setTurretPower(0.0);

        Robot.getInstance().subsystems.add(this);
        lastNanos = System.nanoTime();
    }



    public void enableAutoAim(boolean enable) {
        autoAimEnabled = enable;
        resetPID();
        if (!enable) setTurretPower(0.0);
    }

    public void setTurretPower(double pwr) {
        turret.setPower(clamp(pwr, -1.0, 1.0));
    }

    // PID loop

    @Override
    public void periodic() {
        if (!autoAimEnabled) return;

        long now = System.nanoTime();
        double dt = (now - lastNanos) / 1e9;
        lastNanos = now;

        if (dt < MIN_DT) dt = MIN_DT;
        if (dt > MAX_DT) dt = MAX_DT;


        if (!camera.hasBasket()) {
            setTurretPower(0.0);
            lastErrDeg = 0.0;

            return;
        }

        // Positive error -> target is left -> rotate left (sign depends on your mounting)
        double yawErrDeg = camera.getBasketYawDeg();

        // Deadband to ignore tiny jitter
        if (Math.abs(yawErrDeg) < Constants.deadbandDeg) {
            yawErrDeg = 0.0;
        }

        // Low-pass filter the error (EMA)
        errFiltDeg = Constants.errAlpha * yawErrDeg + (1.0 - Constants.errAlpha) * errFiltDeg;


        integral += errFiltDeg * dt;
        if (yawErrDeg == 0.0) {
            integral *= 0.5;
        }
        integral = clamp(integral, -Constants.maxIntegral, Constants.maxIntegral);


        double deriv = (errFiltDeg - lastErrDeg) / dt;
        deriv = clamp(deriv, -Constants.maxDeriv, Constants.maxDeriv);
        lastErrDeg = errFiltDeg;

        // PID gains map error -> power
        // kP_v: power per degree
        // kI_v: power per (deg·s)
        // kD_v: power per (deg/s)
        double rawPower =
                Constants.kP_v * errFiltDeg
                        + Constants.kI_v * integral
                        + Constants.kD_v * deriv;

        // Optional static feed to overcome stiction if needed (keep small)
        if (yawErrDeg != 0.0 && Constants.kS > 0) {
            rawPower += Math.signum(errFiltDeg) * Constants.kS;
        }

        // Clamp to allowed power range
        double maxPower = clamp(Constants.maxPower, 0.0, 1.0);
        double power = clamp(rawPower, -maxPower, maxPower);

        // Anti-windup (conditional integration): if saturated and still pushing same direction, bleed integral
        if (Math.abs(power) >= maxPower - 1e-6 && Math.signum(power) == Math.signum(rawPower)) {
            // freeze/bleed integrator so it doesn't keep winding while saturated
            integral *= 0.95;
        }

        setTurretPower(power);
    }

    // ---------------- Helpers ----------------

    private void resetPID() {
        integral = 0.0;
        lastErrDeg = 0.0;
        errFiltDeg = 0.0;
        lastNanos = System.nanoTime();
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
