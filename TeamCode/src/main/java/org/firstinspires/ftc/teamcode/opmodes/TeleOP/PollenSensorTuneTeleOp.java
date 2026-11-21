package org.firstinspires.ftc.teamcode.opmodes.TeleOP;

import static org.firstinspires.ftc.teamcode.subsystems.pollensensor.PollenSensorConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.commands.intake.intakeOff;
import org.firstinspires.ftc.teamcode.commands.intake.intakeOn;
import org.firstinspires.ftc.teamcode.commands.intake.intakeReverse;
import org.firstinspires.ftc.teamcode.subsystems.indicator.Indicator;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.pollensensor.PollenSensor;

@Config
@TeleOp(name = "Pollen Sensor Tune", group = "Tuning")
public class PollenSensorTuneTeleOp extends CommandOpMode {
    public static double DISTANCE_STEP_CM = 0.1;
    public static double CLEAR_STEP_S = 0.01;
    public static double TRIGGER_THRESHOLD = 0.5;

    private Intake intake;
    private PollenSensor pollenSensor;
    private Indicator indicator;
    private TelemetryData telemetryData;
    private Intake.State requestedIntake = Intake.State.OFF;
    private final ElapsedTime loopTimer = new ElapsedTime();

    private double min1;
    private double max1;
    private double min2;
    private double max2;

    @Override
    public void initialize() {
        super.reset();

        intake = new Intake(hardwareMap);
        pollenSensor = new PollenSensor(hardwareMap);
        indicator = new Indicator(hardwareMap, null, pollenSensor::isPresent);
        register(intake, pollenSensor, indicator);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryData = new TelemetryData(telemetry);
        resetStats();

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(this::resetStats);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                () -> PRESENT_DISTANCE_CM = Range.clip(PRESENT_DISTANCE_CM + DISTANCE_STEP_CM, 0, 20));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                () -> PRESENT_DISTANCE_CM = Range.clip(PRESENT_DISTANCE_CM - DISTANCE_STEP_CM, 0, 20));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                () -> CLEAR_TIME_S = Range.clip(CLEAR_TIME_S + CLEAR_STEP_S, 0, 1));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                () -> CLEAR_TIME_S = Range.clip(CLEAR_TIME_S - CLEAR_STEP_S, 0, 1));    }

    private void resetStats() {
        min1 = Double.POSITIVE_INFINITY;
        max1 = Double.NEGATIVE_INFINITY;
        min2 = Double.POSITIVE_INFINITY;
        max2 = Double.NEGATIVE_INFINITY;
    }

    @Override
    public void run() {
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        Intake.State wanted = gamepad1.right_trigger > TRIGGER_THRESHOLD ? Intake.State.INTAKE
                : gamepad1.left_trigger > TRIGGER_THRESHOLD ? Intake.State.REVERSE
                : Intake.State.OFF;
        if (wanted != requestedIntake) {
            requestedIntake = wanted;
            schedule(wanted == Intake.State.INTAKE ? new intakeOn(intake)
                    : wanted == Intake.State.REVERSE ? new intakeReverse(intake)
                    : new intakeOff(intake));
        }

        super.run();

        double d1 = pollenSensor.getDistance1();
        double d2 = pollenSensor.getDistance2();
        min1 = Math.min(min1, d1);
        max1 = Math.max(max1, d1);
        min2 = Math.min(min2, d2);
        max2 = Math.max(max2, d2);

        telemetryData.addData("Controls", "hold RT = intake, hold LT = reverse, B = reset min/max");
        telemetryData.addData("Tune controls", "dpad up/down = PRESENT_DISTANCE_CM, dpad right/left = CLEAR_TIME_S");
        telemetryData.addData("Intake state", intake.getState());
        pollenSensor.addTelemetry(telemetryData);
        telemetryData.addData("Sensor 1 min / max cm", min1 + " / " + max1);
        telemetryData.addData("Sensor 2 min / max cm", min2 + " / " + max2);
        indicator.addTelemetry(telemetryData);
        telemetryData.addData("PRESENT_DISTANCE_CM", PRESENT_DISTANCE_CM);
        telemetryData.addData("CLEAR_TIME_S", CLEAR_TIME_S);
        telemetryData.addData("Loop ms", loopMs);
        telemetryData.update();
    }

    @Override
    public void end() {
        intake.setState(Intake.State.OFF);
    }
}
