package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.Control;
import org.firstinspires.ftc.teamcode.control.DriveControl;
import org.firstinspires.ftc.teamcode.control.IntakeControl;
import org.firstinspires.ftc.teamcode.control.VisionControl;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@TeleOp
public class MainTestTeleOp extends OpMode {
    GoBildaPinpointDriver pinpoint;
    Robot robot;
    private JoinedTelemetry joinedTelemetry;

    Gamepad currentGamepad1;
    Gamepad previousGamepad1;

    Gamepad currentGamepad2;
    Gamepad previousGamepad2;
    VisionControl visionControl;
    IntakeControl intakeControl;
    DriveControl driveControl;
    List<Control> controls;

    public ElapsedTime telemetryTimer;

    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        robot = new Robot(hardwareMap, telemetry);
        visionControl = new VisionControl(robot, gamepad1, gamepad2);
        intakeControl = new IntakeControl(robot, gamepad1, gamepad2);
        driveControl = new DriveControl(robot.drive, gamepad1, gamepad2);

        controls = new ArrayList<>(Arrays.asList(visionControl, intakeControl, driveControl));

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();

        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );
        telemetryTimer = new ElapsedTime();
    }

    @Override
    public void init_loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
        telemetry.update();
    }

    @Override
    public void start() {
        robot.toInit();
        telemetryTimer.reset();
    }

    @Override
    public void loop() {
        gamepadUpdate();
        controlsUpdate();
        robot.update();
        controlsTelemetryUpdate();
    }

    @Override
    public void stop() {
        robot.stop();
    }

    public void controlsUpdate() {
        for (Control c: controls) {
            c.update();
        }
    }

    public void controlsTelemetryUpdate() {
        if (telemetryTimer.milliseconds() > 200) {
            for (Control c : controls) {
                c.addTelemetry(joinedTelemetry);
            }
            joinedTelemetry.update();
            telemetryTimer.reset();
        }
    }

    public void gamepadUpdate(){
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
    }

}
