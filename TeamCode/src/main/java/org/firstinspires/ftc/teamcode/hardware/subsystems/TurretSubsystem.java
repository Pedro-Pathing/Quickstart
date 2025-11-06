package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;

public class TurretSubsystem extends RE_SubsystemBase {

    private final DcMotorEx turretMotor;
    private final CameraSubsystem camera;


    private double integral = 0.0;
    private double lastErrDeg = 0.0;
    private double errFiltDeg = 0.0;
    private long lastNanos = 0L;

    public enum TurretState {
        MANUAL,
        AUTO_AIM
    }

    private TurretState turretState;

    // Time clamps
    private static final double MIN_DT = 1e-3;  // 1 ms
    private static final double MAX_DT = 0.05;  // 50 ms


    private static final double TICKS_PER_REV = 1440.0;
    private static final double GEAR_RATIO = 208.0 / 60.0;
    private static final double TICKS_PER_DEGREE = (TICKS_PER_REV * GEAR_RATIO) / 360.0;

    private static final double LEFT_LIMIT_DEG = -90.0;
    private static final double RIGHT_LIMIT_DEG = 90.0;

    public TurretSubsystem(HardwareMap hw, String motorName, CameraSubsystem camera) {
        this.turretMotor = hw.get(DcMotorEx.class, motorName);
        this.camera = camera;

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretState = TurretState.MANUAL;
        setTurretPower(0.0);

        Robot.getInstance().subsystems.add(this);
        lastNanos = System.nanoTime();
    }

    // ---------------- State Control ----------------

    public void setTurretState(TurretState state) {
        this.turretState = state;
        if (state == TurretState.MANUAL) {
            setTurretPower(0.0);
        } else {
            resetPID();
        }
    }

    public TurretState getTurretState() {
        return turretState;
    }

    public void enableAutoAim(boolean enable) {
        setTurretState(enable ? TurretState.AUTO_AIM : TurretState.MANUAL);
    }

    // ---------------- Power & Motion ----------------

    public void setTurretPower(double pwr) {
        double angleDeg = getTurretAngleDeg();

        // Clamp power if near angle limits
        if (angleDeg <= LEFT_LIMIT_DEG && pwr < 0) {
            pwr = 0;
        } else if (angleDeg >= RIGHT_LIMIT_DEG && pwr > 0) {
            pwr = 0;
        }

        turretMotor.setPower(clamp(pwr, -1.0, 1.0));
    }

    public double getTurretAngleDeg() {
        return turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    @Override
    public void updateData() {
        Robot.getInstance().data.turretState = turretState.name();
        Robot.getInstance().data.turretAngleDeg = getTurretAngleDeg();
    }

    @Override
    public void periodic() {
        if (turretState == TurretState.AUTO_AIM) {
            runAutoAimPID();
        }
    }


    private void runAutoAimPID() {
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

        double yawErrDeg = camera.getBasketYawDeg();


        if (Math.abs(yawErrDeg) < Constants.deadbandDeg) yawErrDeg = 0.0;


        errFiltDeg = Constants.errAlpha * yawErrDeg + (1.0 - Constants.errAlpha) * errFiltDeg;

        integral += errFiltDeg * dt;
        if (yawErrDeg == 0.0) integral *= 0.5;
        integral = clamp(integral, -Constants.maxIntegral, Constants.maxIntegral);

        double deriv = (errFiltDeg - lastErrDeg) / dt;
        deriv = clamp(deriv, -Constants.maxDeriv, Constants.maxDeriv);
        lastErrDeg = errFiltDeg;

        double rawPower =
                Constants.kP_v * errFiltDeg +
                        Constants.kI_v * integral +
                        Constants.kD_v * deriv;

        if (yawErrDeg != 0.0 && Constants.kS > 0) {
            rawPower += Math.signum(errFiltDeg) * Constants.kS;
        }

        double maxPower = clamp(Constants.maxPower, 0.0, 1.0);
        double power = clamp(rawPower, -maxPower, maxPower);


        if (Math.abs(power) >= maxPower - 1e-6 && Math.signum(power) == Math.signum(rawPower)) {
            integral *= 0.95;
        }

        setTurretPower(power);
    }



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
