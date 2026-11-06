package org.firstinspires.ftc.teamcode.opmodes.TeleOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.commands.intake.intakeOff;
import org.firstinspires.ftc.teamcode.commands.intake.intakeOn;
import org.firstinspires.ftc.teamcode.commands.intake.intakeReverse;
import org.firstinspires.ftc.teamcode.commands.intake.intakeToShooter;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;

import java.util.Locale;
import java.util.function.DoubleSupplier;

@Config
@TeleOp(name = "Shooter Tune", group = "Tuning")
public class ShooterTuneTeleOp extends CommandOpMode {
    public static double RPM_STEP = 50;
    public static double TOLERANCE_STEP = 5;

    private Shooter shooter;
    private Intake intake;
    private TelemetryData telemetryData;

    private boolean shooterOn = false;
    private final ElapsedTime clock = new ElapsedTime();
    private boolean wasReady = false;
    private boolean dipping = false;
    private double dipStart = 0;
    private double dipMinRpm = 0;
    private double lastDropRpm = Double.NaN;
    private double lastRecoveryS = Double.NaN;
    private int shots = 0;

    @Override
    public void initialize() {
        super.reset();

        shooter = new Shooter(hardwareMap, () -> Double.NaN);
        intake = new Intake(hardwareMap);
        register(shooter, intake);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryData = new TelemetryData(telemetry);

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenActive(new intakeOn(intake));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenInactive(new intakeOff(intake));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenActive(new intakeToShooter(intake));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenInactive(new intakeOff(intake));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenActive(new intakeReverse(intake));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenInactive(new intakeOff(intake));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> shooterOn = !shooterOn);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                () -> ShooterConstants.TUNING_RPM += RPM_STEP);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                () -> ShooterConstants.TUNING_RPM = Math.max(0, ShooterConstants.TUNING_RPM - RPM_STEP));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                () -> ShooterConstants.SHOOTER_TOLERANCE_RPM += TOLERANCE_STEP);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                () -> ShooterConstants.SHOOTER_TOLERANCE_RPM = Math.max(0, ShooterConstants.SHOOTER_TOLERANCE_RPM - TOLERANCE_STEP));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
            shots = 0;
            lastDropRpm = Double.NaN;
            lastRecoveryS = Double.NaN;
        });
    }

    @Override
    public void run() {
        shooter.setFlywheel(shooterOn ? ShooterConstants.TUNING_RPM : 0, shooterOn);

        super.run();

        double target = shooter.getTargetRPM();
        double rpm = shooter.getRPM();
        boolean ready = shooter.flywheelReady();
        double now = clock.seconds();
        if (shooterOn && wasReady && !dipping && rpm < target - ShooterConstants.SHOOTER_TOLERANCE_RPM) {
            dipping = true;
            dipStart = now;
            dipMinRpm = rpm;
        }
        if (dipping) {
            dipMinRpm = Math.min(dipMinRpm, rpm);
            if (ready) {
                dipping = false;
                shots++;
                lastDropRpm = target - dipMinRpm;
                lastRecoveryS = now - dipStart;
            } else if (!shooterOn) {
                dipping = false;
            }
        }
        wasReady = ready;

        telemetryData.addData("Controls", "A = shooter ON/OFF, dpad up/down = RPM +/- RPM_STEP, dpad right/left = tolerance +/- TOLERANCE_STEP, LB = intake, RB = shoot, Y = reverse, X = reset shot stats");
        telemetryData.addData("Shooter", shooterOn ? "ON" : "OFF");
        telemetryData.addData("Target RPM", target);
        telemetryData.addData("Current RPM", rpm);
        telemetryData.addData("Error RPM", shooterOn ? target - rpm : 0);
        telemetryData.addData("Ready", ready);
        telemetryData.addData("Tolerance RPM", ShooterConstants.SHOOTER_TOLERANCE_RPM);
        telemetryData.addData("Ready window RPM", String.format(Locale.US, "%.0f to %.0f",
                target - ShooterConstants.SHOOTER_TOLERANCE_RPM, target + ShooterConstants.SHOOTER_TOLERANCE_RPM));
        telemetryData.addData("Shots detected", shots);
        telemetryData.addData("Last shot RPM drop", lastDropRpm);
        telemetryData.addData("Last shot recovery s", lastRecoveryS);
        telemetryData.addData("TUNING_RPM", ShooterConstants.TUNING_RPM);
        telemetryData.addData("Flywheel PIDF", ShooterConstants.FLYWHEEL_PIDF_COEFFICIENTS);
        telemetryData.update();
    }

    @Override
    public void end() {
        intake.intakeOff();
    }
}
