package org.firstinspires.ftc.teamcode.opmodes.TeleOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.commands.intake.intakeOff;
import org.firstinspires.ftc.teamcode.commands.intake.intakeOn;
import org.firstinspires.ftc.teamcode.commands.intake.intakeReverse;
import org.firstinspires.ftc.teamcode.subsystems.gate.Gate;
import org.firstinspires.ftc.teamcode.subsystems.gate.GateConstants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intakeFlower.IntakeFlower;
import org.firstinspires.ftc.teamcode.subsystems.intakeFlower.IntakeFlowerConstants;

@Config
@TeleOp(name = "Flower + Gate Tune", group = "Tuning")
public class IntakeGateTuneTeleOp extends CommandOpMode {
    public static double FLOWER_STEP = 0.01;
    public static double GATE_STEP = 0.01;
    public static double TRIGGER_THRESHOLD = 0.5;

    private IntakeFlower flower;
    private Gate gate;
    private Intake intake;
    private TelemetryData telemetryData;
    private Intake.State requestedIntake = Intake.State.OFF;

    @Override
    public void initialize() {
        super.reset();

        flower = new IntakeFlower(hardwareMap);
        gate = new Gate(hardwareMap);
        intake = new Intake(hardwareMap);
        register(flower, gate, intake);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryData = new TelemetryData(telemetry);

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> flower.setState(IntakeFlower.State.TUNE));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> {
            IntakeFlowerConstants.IntakeFlowerTune = Range.clip(IntakeFlowerConstants.IntakeFlowerTune + FLOWER_STEP, 0, 1);
            flower.setState(IntakeFlower.State.TUNE);
        });
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> {
            IntakeFlowerConstants.IntakeFlowerTune = Range.clip(IntakeFlowerConstants.IntakeFlowerTune - FLOWER_STEP, 0, 1);
            flower.setState(IntakeFlower.State.TUNE);
        });
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> flower.setState(IntakeFlower.State.DOWN));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> flower.setState(IntakeFlower.State.UP));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(gate::tuning);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> {
            GateConstants.TUNING_POSITION = Range.clip(GateConstants.TUNING_POSITION + GATE_STEP, 0, 1);
            gate.tuning();
        });
        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> {
            GateConstants.TUNING_POSITION = Range.clip(GateConstants.TUNING_POSITION - GATE_STEP, 0, 1);
            gate.tuning();
        });
        gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(gate::open);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(gate::close);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.START).whenPressed(
                () -> GateConstants.OPEN_POSITION = GateConstants.TUNING_POSITION);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(
                () -> GateConstants.CLOSED_POSITION = GateConstants.TUNING_POSITION);
    }

    @Override
    public void run() {
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

        telemetryData.addData("Intake controls", "hold RT = intake ON, hold LT = intake REVERSE, release = OFF");
        telemetryData.addData("Intake state", intake.getState());
        telemetryData.addData("Flower controls", "A = TUNE, dpad up/down = tune position +/- FLOWER_STEP, dpad right = DOWN, dpad left = UP");
        telemetryData.addData("Gate controls", "B = TUNING, RB/LB = tuning position +/- GATE_STEP, X = OPEN, Y = CLOSED, "
                + "START = save tuning as OPEN, BACK = save tuning as CLOSED");
        telemetryData.addData("Flower state", flower.getState());
        telemetryData.addData("Flower position", flower.getPosition());
        telemetryData.addData("IntakeFlowerTune", IntakeFlowerConstants.IntakeFlowerTune);
        telemetryData.addData("IntakeFlowerOn1 (DOWN)", IntakeFlowerConstants.IntakeFlowerOn1);
        telemetryData.addData("IntakeFlowerOff (UP)", IntakeFlowerConstants.IntakeFlowerOff);
        gate.addTelemetry(telemetryData);
        telemetryData.addData("TUNING_POSITION", GateConstants.TUNING_POSITION);
        telemetryData.addData("OPEN_POSITION", GateConstants.OPEN_POSITION);
        telemetryData.addData("CLOSED_POSITION", GateConstants.CLOSED_POSITION);
        telemetryData.update();
    }

    @Override
    public void end() {
        intake.setState(Intake.State.OFF);
    }
}
