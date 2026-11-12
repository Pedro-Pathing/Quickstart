package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechanisms.ServoUtil;

/**
 * ServoTestTeleOp - A minimal TeleOp program for testing the pollen and nectar gate servos
 *
 * TO ADJUST POSITIONS:
 * Edit these constants in RobotConstants:
 * - RobotConstants.NECTAR_SERVO_OPEN_POSITION / RobotConstants.NECTAR_SERVO_CLOSED_POSITION
 * - RobotConstants.POLLEN_SERVO_OPEN_POSITION / RobotConstants.POLLEN_SERVO_CLOSED_POSITION
 *
 * CONTROLS:
 * - gamepad1.x: Toggle nectar gate open/closed
 * - gamepad1.b: Toggle pollen gate open/closed
 */
@TeleOp(name = "Servo Test", group = "Test")
@Disabled
public class ServoTestTeleOp extends OpMode {

    private ServoUtil servos;

    @Override
    public void init() {
        servos = new ServoUtil();
        servos.init(hardwareMap);

        telemetry.addData("Status", "Servo Test TeleOp Ready");
        telemetry.addData("Controls", "X=toggle nectar, B=toggle pollen");
        telemetry.update();
    }

    @Override
    public void start() {
        servos.open();
    }

    @Override
    public void loop() {
        if (gamepad1.xWasPressed()) {
            servos.toggleNectar();
        }
        if (gamepad1.bWasPressed()) {
            servos.togglePollen();
        }

        updateTelemetry();
    }

    /**
     * Update telemetry display
     */
    private void updateTelemetry() {
        telemetry.addData("Nectar", servos.isNectarOpen() ? "OPEN" : "CLOSED");
        telemetry.addData("Nectar Position", String.format("%.2f", servos.getNectarPosition()));
        telemetry.addData("Pollen", servos.isPollenOpen() ? "OPEN" : "CLOSED");
        telemetry.addData("Pollen Position", String.format("%.2f", servos.getPollenPosition()));
        telemetry.addData("---", "---");
        telemetry.addData("Controls", "X=toggle nectar, B=toggle pollen");
        telemetry.update();
    }
}
