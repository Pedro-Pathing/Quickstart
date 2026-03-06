package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.common.Processor;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.command.IntakeBall;
import org.firstinspires.ftc.teamcode.common.command.IntakeSort;
import org.firstinspires.ftc.teamcode.common.command.IntakeTransfer;
import org.firstinspires.ftc.teamcode.common.command.OuttakeBall;
import org.firstinspires.ftc.teamcode.common.command.Reset;
import org.firstinspires.ftc.teamcode.common.command.Shoot;
import org.firstinspires.ftc.teamcode.common.command.Sort;
import org.firstinspires.ftc.teamcode.common.controller.Button;
import org.firstinspires.ftc.teamcode.common.controller.ButtonListener;
import org.firstinspires.ftc.teamcode.common.enums.Color;

import java.util.Objects;

@TeleOp
public class DriverControl extends LinearOpMode {
    private Robot robot;

    private Boolean vibrate;


    private Processor processor;


    @Override
    public void runOpMode() throws InterruptedException {
        ButtonListener listener = new ButtonListener(gamepad1);
        robot = new Robot(hardwareMap);
        processor = new Processor();
        vibrate = true;

        listener.addListener(Button.R_TRIGGER_DOWN, () -> {
            gamepad1.rumble(500);
            if (Robot.sort && (Robot.motif == 21 || Robot.motif == 22 || Robot.motif == 23)) {
                processor.override(new Sort(Robot.motif));
            } else {
                processor.override(new Shoot());
                vibrate = true;
            }

        });
        listener.addListener(Button.R_BUMPER_DOWN, () -> {
            Robot.sort = !Robot.sort;
        });
        listener.addListener(Button.L_TRIGGER_DOWN, () -> {
            if (Robot.sort) {
                processor.override(new IntakeSort());
            } else {
                processor.override(new IntakeBall(true));
            }

        });

        listener.addListener(Button.L_BUMPER_DOWN, () -> {
            processor.override(new OuttakeBall());
        });
        listener.addListener(Button.SQUARE_DOWN, () -> {
            processor.override(new IntakeBall(false));
        });

        listener.addListener(Button.L_STICK_DOWN, () -> {
            if (robot.turret.isUseAutoAim()) {
                robot.turret.noAutoAim();
            } else {
                robot.turret.useAutoAim();
            }

        });
        listener.addListener(Button.R_STICK_DOWN, () -> {
            if (robot.shooter.isUseAutoAim()) {
                robot.shooter.noAutoAim();
            } else {
                robot.shooter.useAutoAim();
            }

        });
        listener.addListener(Button.CROSS_DOWN, () -> {
            processor.override(new Reset());
        });
        listener.addListener(Button.CIRCLE_DOWN, () -> {
            processor.override(new IntakeTransfer());
        });

        listener.addListener(Button.TRIANGLE_DOWN, () -> {
            robot.follower.startTeleOpDrive(true);
        });


        Robot.sort = false;
        if (Robot.robotPos != null) {
            robot.follower.setStartingPose(Robot.robotPos);
        } else {
            robot.follower.setStartingPose(new Pose(117, 132, Math.toRadians(315)));
        }
        robot.follower.update();
        while (opModeInInit()) {
            robot.limelight.setPipLine(null);
            if (robot.limelight.getMotif() != 0) {
                Robot.motif = robot.limelight.getMotif();
            }
            robot.limelight.update();
        }
        waitForStart();


        robot.follower.setMaxPower(1);
        robot.follower.startTeleOpDrive(false);

        while (!isStopRequested()) {
            if (Robot.threeBalls && vibrate == true && processor.getCommand() instanceof IntakeBall) {
                gamepad1.rumble(500);
                vibrate = false;
            }

            if (processor.getLastExecuted() instanceof Shoot && !processor.isBusy()) {
                processor.override(new IntakeBall(true));
                robot.follower.startTeleOpDrive(false);
            }
            if (processor.getLastExecuted() instanceof Sort && !processor.isBusy()) {
                processor.override(new IntakeSort());
            }
            /*if (!processor.isBusy() && Robot.threeBalls && (robot.inBackTriangle() || robot.inFrontTriangle())){
                gamepad1.rumble(500);
                processor.override(new Shoot());
                vibrate = true;
            }*/

            robot.follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            robot.limelight.setPipLine(Robot.color);
            robot.update();
            listener.update();
            processor.update(robot);

            telemetry.addData("BallColors", robot.colorSensors.getColors());
            telemetry.addData("colorSensorBall", robot.colorSensors.ballsInIntake());
            telemetry.addData("intakeCurrent", robot.intake.getCurrent());
            telemetry.addData("intakeVelocity", robot.intake.getVelocity());
            telemetry.addData("vibrate", vibrate);


            telemetry.addData("robotPos", Robot.robotPos);
            telemetry.addData("turretAngle", robot.turret.getTurretAngle());
            telemetry.addData("sort", Robot.sort);
            telemetry.addData("distance", robot.limelight.getDistance());
            telemetry.addData("velocity", robot.shooter.getVelocity());
            telemetry.addData("targetVelocity", robot.shooter.getTargetVelocity());


            telemetry.update();
        }
    }
}
