package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.TurretSubsystem;

@TeleOp(name = "Turret Test (CR Servo)", group = "Test")
public class TurretTestOpMode extends OpMode {

    private CameraSubsystem camera;
    private TurretSubsystem turret;

    private boolean autoAimEnabled = true;

    // edge-detect state
    private boolean prevA, prevB;
    private boolean prevLeftBumper, prevRightBumper;

    // manual control params (CR servo = power control)
    private double manualMaxPower = 0.6;   // scale for right stick → power
    private double nudgePower    = 0.4;    // bumper “burst” power
    private long   nudgeUntilNs  = 0L;     // nudge end time (ns)
    private double nudgeSign     = 0.0;    // -1 for LB, +1 for RB

    private long lastNanos;

    @Override
    public void init() {
        camera = new CameraSubsystem(hardwareMap, "limelight");
        turret = new TurretSubsystem(hardwareMap, "turretServo", camera);

        turret.enableAutoAim(autoAimEnabled);

        lastNanos = System.nanoTime();
        telemetry.addLine("Turret Test (CR Servo): initialized");
        telemetry.addLine("A = AutoAim ON,  B = AutoAim OFF");
        telemetry.addLine("Right stick X = manual power (when AutoAim OFF)");
        telemetry.addLine("LB/RB = quick nudge burst (when AutoAim OFF)");
    }

    @Override
    public void start() {
        lastNanos = System.nanoTime();
    }

    @Override
    public void loop() {
        long now = System.nanoTime();
        double dt = (now - lastNanos) / 1e9;
        lastNanos = now;
        if (dt <= 0) dt = 1e-3;

        // --- Auto-aim toggle ---
        if (rising(gamepad1.a, prevA)) {
            autoAimEnabled = true;
            turret.enableAutoAim(true);
        }
        if (rising(gamepad1.b, prevB)) {
            autoAimEnabled = false;
            turret.enableAutoAim(false);
            // stop the turret when exiting auto
            turret.setTurretPower(0.0);
            nudgeUntilNs = 0L;
            nudgeSign = 0.0;
        }

        // --- Manual steering when auto-aim is OFF ---
        if (!autoAimEnabled) {
            // Bumper nudges: brief power bursts to “tap” the turret
            if (rising(gamepad1.left_bumper, prevLeftBumper)) {
                nudgeSign = -1.0;
                nudgeUntilNs = now + 200_000_000L; // 200 ms
            }
            if (rising(gamepad1.right_bumper, prevRightBumper)) {
                nudgeSign = +1.0;
                nudgeUntilNs = now + 200_000_000L; // 200 ms
            }

            double stick = gamepad1.right_stick_x; // -1..1
            double basePower = stick * manualMaxPower;

            // If in a nudge window, override with burst power
            if (now < nudgeUntilNs) {
                basePower = nudgeSign * nudgePower;
            } else {
                nudgeSign = 0.0;
            }

            turret.setTurretPower(basePower);
        }

        // --- Telemetry ---
        telemetry.addData("AutoAim", autoAimEnabled);
        telemetry.addData("Manual stick X", "%.2f", gamepad1.right_stick_x);
        telemetry.addData("Nudge active", now < nudgeUntilNs);
        telemetry.addData("Has Basket", camera.hasBasket());
        telemetry.addData("Yaw Error (deg)", "%.2f", camera.getBasketYawDeg());
        telemetry.addData("Est Dist (m)", "%.2f", camera.getBasketDistanceM());
        telemetry.addData("Shoot Range", camera.getShootDistance());
        telemetry.update();

        // --- Update previous states ---
        prevA = gamepad1.a; prevB = gamepad1.b;
        prevLeftBumper = gamepad1.left_bumper; prevRightBumper = gamepad1.right_bumper;
    }

    private boolean rising(boolean now, boolean prev) {
        return now && !prev;
    }
}
