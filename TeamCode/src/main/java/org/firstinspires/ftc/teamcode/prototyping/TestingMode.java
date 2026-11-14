
package org.firstinspires.ftc.teamcode.prototyping;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Hardware Check")
public class TestingMode extends OpMode {
    static final String[] MOTORS = {
            "left_front_drive",
            "right_front_drive",
            "left_back_drive",
            "right_back_drive",
            "left_intake",
            "right_intake",
            "left_flywheel",
            "right_flywheel"
    };

    static final String[] CR_SERVOS = {
            "left_feeder",
            "right_feeder"
    };

    static final String[] SERVOS = {
            "blocker"
    };

    List<String> names = new ArrayList<>();
    List<HardwareDevice> devices = new ArrayList<>();
    List<String> missing = new ArrayList<>();

    int selected = 0;

    @Override
    public void init() {
        for (String name : MOTORS) {
            add(name, hardwareMap.tryGet(DcMotor.class, name));
        }

        for (String name : CR_SERVOS) {
            add(name, hardwareMap.tryGet(CRServo.class, name));
        }

        for (String name : SERVOS) {
            add(name, hardwareMap.tryGet(Servo.class, name));
        }
    }

    @Override
    public void init_loop() {
        telemetry.addData("Found", names.toString());
        telemetry.addData(
                "Missing",
                missing.isEmpty() ? "none" : missing.toString()
        );
        telemetry.update();
    }

    @Override
    public void loop() {
        if (devices.isEmpty()) {
            telemetry.addLine("Nothing found in the config");
            telemetry.update();
            return;
        }

        boolean down = gamepad1.dpadDownWasPressed();
        boolean up = gamepad1.dpadUpWasPressed();

        if (down || up) {
            stopAll();

            selected = (selected + (down ? 1 : -1)
                    + devices.size()) % devices.size();
        }

        HardwareDevice device = devices.get(selected);
        double stick = -gamepad1.left_stick_y;

        telemetry.addLine("Dpad UP/DOWN: Select device");
        telemetry.addLine("Left stick: Test selected device");
        telemetry.addData("Selected", names.get(selected));

        if (device instanceof DcMotor) {
            DcMotor motor = (DcMotor) device;

            motor.setPower(stick);

            telemetry.addData("Power", "%.2f", stick);
            telemetry.addData(
                    "Encoder",
                    motor.getCurrentPosition()
            );

        } else if (device instanceof CRServo) {
            CRServo servo = (CRServo) device;

            servo.setPower(stick);

            telemetry.addData("Power", "%.2f", stick);

        } else if (device instanceof Servo) {
            Servo servo = (Servo) device;

            double position = (stick + 1.0) / 2.0;
            servo.setPosition(position);

            telemetry.addData("Position", "%.2f", position);
        }

        telemetry.addData(
                "Missing",
                missing.isEmpty() ? "none" : missing.toString()
        );

        telemetry.update();
    }

    @Override
    public void stop() {
        stopAll();
    }

    void add(String name, HardwareDevice device) {
        if (device == null) {
            missing.add(name);
        } else {
            names.add(name);
            devices.add(device);
        }
    }

    void stopAll() {
        for (HardwareDevice device : devices) {
            if (device instanceof DcMotor) {
                ((DcMotor) device).setPower(0);
            }

            if (device instanceof CRServo) {
                ((CRServo) device).setPower(0);
            }
        }
    }
}
