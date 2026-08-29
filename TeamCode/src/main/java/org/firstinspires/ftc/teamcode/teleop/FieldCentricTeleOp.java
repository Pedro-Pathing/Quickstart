package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@TeleOp(name = "Field Centric", group = "TeleOp")
public class FieldCentricTeleOp extends OpMode {

    private Drivetrain drivetrain;

    @Override
    public void init() {

        drivetrain = new Drivetrain(hardwareMap);

        telemetry.addLine("Drivetrain initialized");
        telemetry.update();
    }

    @Override
    public void start() {

        drivetrain.start();
    }

    @Override
    public void loop() {

        // Update localization FIRST
        drivetrain.update();

        // Get current robot pose
        Pose pose = drivetrain.getPose();

        // Gamepad input
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        // Robot heading in radians
        double heading = pose.getHeading();

        // Field-centric transformation
        double robotForward =
                forward * Math.cos(heading)
                        + strafe * Math.sin(heading);

        double robotStrafe =
                -forward * Math.sin(heading)
                        + strafe * Math.cos(heading);

        // Drive using robot-relative values
        drivetrain.drive(
                robotForward,
                robotStrafe,
                turn
        );

        // Telemetry
        telemetry.addData("X", "%.2f", pose.getX());
        telemetry.addData("Y", "%.2f", pose.getY());
        telemetry.addData(
                "Heading",
                "%.1f°",
                Math.toDegrees(heading)
        );

        telemetry.update();
    }

    @Override
    public void stop() {
        drivetrain.stop();
    }
}