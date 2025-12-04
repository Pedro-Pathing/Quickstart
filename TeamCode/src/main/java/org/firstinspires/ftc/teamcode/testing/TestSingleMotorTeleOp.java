package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Motor Power", group="Test OpModes")
public class TestSingleMotorTeleOp extends OpMode {

    private List<DcMotor> motors = new ArrayList<>();
    private DcMotor motor;
    private int selectedIndex = 0;
    private double power = 0.5;
    private boolean brake = false;
    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad lastGamepad1 = new Gamepad();

    @Override
    public void init() {
        // store motors currently accessible from the HardwareMap
        motors.clear();
        motors.addAll(hardwareMap.getAll(DcMotor.class));
    }

    @Override
    public void init_loop() {

        if (motors.isEmpty()) {
            telemetry.addLine("no motors found");
            telemetry.update();
            return;
        }

        lastGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        // Rising Edge Detector for X --> cycle through motors
        if (currentGamepad1.x && !lastGamepad1.x) {
            selectedIndex++;
            if (selectedIndex >= motors.size()) {
                selectedIndex = 0;
            }
        }

        // Handle telemetry
        telemetry.addData("Selected Motor Port", motors.get(selectedIndex).getPortNumber());
        telemetry.addLine("Press X to select the next motor.");
        telemetry.addData("motor count", motors.size());
        telemetry.update();
    }

    @Override
    public void start() {
        // Store final selection
        motor = motors.get(selectedIndex);
    }

    @Override
    public void loop() {
        lastGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        // Rising Edge Detector for D-pad --> change power
        if (currentGamepad1.dpad_up && !lastGamepad1.dpad_up) {
            power +=.1;
        } else if (currentGamepad1.dpad_down && !lastGamepad1.dpad_down) {
            power -=.1;
        }

        // Rising Edge Detector for Right-Joystick-Y --> actually run motor
        if (currentGamepad1.right_stick_y > .1) {
            motor.setPower(power);
        } else if (currentGamepad1.right_stick_y < -.1) {
            motor.setPower(-power);
        } else {
            motor.setPower(0);
        }

        // Rising Edge Detector for Y --> change brake behavior
        if (currentGamepad1.y && !lastGamepad1.y) {
            brake = !brake;
            motor.setZeroPowerBehavior(brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
        }

        // Handle telemetry
        telemetry.addLine("Right Joystick to run");
        telemetry.addLine("D-pad to change power (up/down +-.1)");
        telemetry.addLine("Y to change brake behavior");
        telemetry.addLine("");
        telemetry.addLine("Motor")
                .addData("Type", motor.getMotorType().getName())
                .addData("Power", "%.3f", motor.getPower())
                .addData("Brake?", brake);

        telemetry.update();
    }

}
