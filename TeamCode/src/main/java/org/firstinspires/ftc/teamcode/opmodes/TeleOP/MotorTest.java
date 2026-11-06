package org.firstinspires.ftc.teamcode.opmodes.TeleOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp(name = "Motor Test", group = "Tuning")
public class MotorTest extends LinearOpMode {
    public static String MOTOR_NAME = "index1";
    public static double POWER = 1;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        waitForStart();
        while (opModeIsActive()) {
            double power = gamepad1.a ? POWER : 0;
            motor.setPower(power);

            telemetry.addData("Controls", "hold A to run MOTOR_NAME at POWER");
            telemetry.addData("Motor", MOTOR_NAME + " (port " + motor.getPortNumber() + ")");
            telemetry.addData("Commanded power", power);
            telemetry.addData("Motor power read back", motor.getPower());
            telemetry.addData("Run mode", motor.getMode());
            telemetry.addData("Encoder position ticks", motor.getCurrentPosition());
            telemetry.addData("Encoder velocity ticks/s", motor.getVelocity());
            telemetry.addData("Current amps", motor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Over current", motor.isOverCurrent());
            telemetry.addData("Battery volts", battery.getVoltage());
            telemetry.update();
        }
    }
}
