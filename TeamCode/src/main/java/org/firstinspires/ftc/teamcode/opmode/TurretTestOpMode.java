package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.TurretSubsystem;

@TeleOp(name = "Turret Test", group = "Test")
public class TurretTestOpMode extends OpMode {

    private CameraSubsystem camera;
    private TurretSubsystem turret;

    private boolean autoAimEnabled = true;

    private boolean prevA, prevB;
    private boolean prevLeftBumper, prevRightBumper;
    private boolean prevLeftStick, prevRightStick;

    private double manualDegPerSec = 120.0;
    private long lastNanos;

    @Override
    public void init() {
        camera = new CameraSubsystem(hardwareMap, "limelight");
        turret = new TurretSubsystem(hardwareMap, "turretServo", camera);

        turret.enableAutoAim(autoAimEnabled);

        lastNanos = System.nanoTime();
        telemetry.addLine("Turret Test: initialized");
        telemetry.addLine("A = AutoAim ON,  B = AutoAim OFF");
        telemetry.addLine("Right stick X = manual steer (when AutoAim OFF)");
        telemetry.addLine("LB/RB = ±5° quick nudge (when AutoAim OFF)");
        telemetry.addLine("LS = recenter to 0°, RS = flip 180°");
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
        }

        // --- Manual steering when auto-aim is OFF ---
        if (!autoAimEnabled) {
            double stick = gamepad1.right_stick_x;
            if (Math.abs(stick) > 0.02) {
                double delta = stick * manualDegPerSec * dt;
                turret.setTurretDeg(turret.getTurretDeg() + delta);
            }

            if (rising(gamepad1.left_bumper, prevLeftBumper)) {
                turret.setTurretDeg(turret.getTurretDeg() - 5.0);
            }
            if (rising(gamepad1.right_bumper, prevRightBumper)) {
                turret.setTurretDeg(turret.getTurretDeg() + 5.0);
            }
        }

        // --- Utility buttons ---
        if (rising(gamepad1.left_stick_button, prevLeftStick)) {
            turret.setTurretDeg(0.0); // recenter
        }
        if (rising(gamepad1.right_stick_button, prevRightStick)) {
            turret.setTurretDeg(turret.getTurretDeg() + 180.0); // flip
        }

        // --- Telemetry ---
        telemetry.addData("AutoAim", autoAimEnabled);
        telemetry.addData("Manual stick X", "%.2f", gamepad1.right_stick_x);
        telemetry.addData("Has Basket", camera.hasBasket());
        telemetry.addData("Yaw Error (deg)", camera.getBasketYawDeg());
        telemetry.addData("Est Dist (m)", camera.getBasketDistanceM());
        telemetry.addData("Shoot Range", camera.getShootDistance());
        telemetry.addData("Turret Deg", "%.1f", turret.getTurretDeg());
        telemetry.update();

        // --- Update previous states ---
        prevA = gamepad1.a; prevB = gamepad1.b;
        prevLeftBumper = gamepad1.left_bumper; prevRightBumper = gamepad1.right_bumper;
        prevLeftStick = gamepad1.left_stick_button; prevRightStick = gamepad1.right_stick_button;
    }

    private boolean rising(boolean now, boolean prev) {
        return now && !prev;
    }
}
