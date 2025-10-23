package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;

public class TurretSubsystem extends RE_SubsystemBase {

    private final Servo turret;
    private final CameraSubsystem camera;

    private double integral;
    private double lastErrorDeg;
    private long lastNanos;

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

        if (!camera.hasBasket()) {
            resetPID();
            return;
        }

        // positive err menas that the target is left so itll turn left
        double yawErrDeg = camera.getBasketYawDeg();

        // small deadband to avoid hunting
        double deadband = 0.3; // deg
        if (Math.abs(yawErrDeg) < deadband) yawErrDeg = 0.0;

        // PID
        long now = System.nanoTime();
        double dt = (now - lastNanos) / 1e9;
        lastNanos = now;
        if (dt <= 0) dt = 1e-3;

        integral += yawErrDeg * dt;
        double deriv = (yawErrDeg - lastErrorDeg) / dt;
        lastErrorDeg = yawErrDeg;

        double correctionDeg = Constants.kP*yawErrDeg + Constants.kI*integral + Constants.kD*deriv;

        double newDeg = getTurretDeg() + correctionDeg;
        setTurretDeg(newDeg);
    }


    private void resetPID() {
        integral = 0;
        lastErrorDeg = 0;
        lastNanos = System.nanoTime();
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private double mapDegToPos(double deg) {

        double spanDeg = (Constants.maxDeg - Constants.minDeg);
        double spanPos = (Constants.maxPos - Constants.minPos);
        double t = (deg - Constants.minDeg) / spanDeg;
        return Constants.minPos + t * spanPos;
    }

    private double mapPosToDeg(double pos) {
        double spanDeg = (Constants.maxDeg - Constants.minDeg);
        double spanPos = (Constants.maxPos - Constants.minPos);
        double t = (pos - Constants.minPos) / spanPos;
        return Constants.minDeg + t * spanDeg;
    }
}
