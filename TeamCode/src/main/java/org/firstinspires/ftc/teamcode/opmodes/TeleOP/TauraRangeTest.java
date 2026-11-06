package org.firstinspires.ftc.teamcode.opmodes.TeleOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;

@Config
@TeleOp(name = "Taura Range Test", group = "Tuning")
public class TauraRangeTest extends LinearOpMode {
    public static String SERVO_NAME = "turret";
    public static double POSITION = 0.5;
    public static double MEASURED_TURRET_DEG = 0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Servo servo = hardwareMap.get(Servo.class, SERVO_NAME);

        double posA = Double.NaN, posB = Double.NaN;

        waitForStart();
        while (opModeIsActive()) {
            servo.setPosition(POSITION);
            if (gamepad1.a) posA = POSITION;
            if (gamepad1.b) posB = POSITION;
            double deltaPosition = posB - posA;
            double servoRangeDeg = MEASURED_TURRET_DEG * TurretConstants.GEAR_RATIO / Math.abs(deltaPosition);

            telemetry.addData("Controls", "set POSITION on Dashboard, A = save point A, B = save point B");
            telemetry.addData("Position", POSITION);
            telemetry.addData("Point A position", posA);
            telemetry.addData("Point B position", posB);
            telemetry.addData("Position change B - A", deltaPosition);
            telemetry.addData("Measured turret deg (protracto)", MEASURED_TURRET_DEG);
            telemetry.addData("SERVO_RANGE_DEG", servoRangeDeg);
            telemetry.update();
        }
    }
}
