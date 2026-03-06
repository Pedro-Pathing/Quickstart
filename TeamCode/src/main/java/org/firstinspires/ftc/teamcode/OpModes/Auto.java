package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Processor;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.command.IntakeBall;
import org.firstinspires.ftc.teamcode.common.command.IntakeTransfer;
import org.firstinspires.ftc.teamcode.common.command.Shoot;
import org.firstinspires.ftc.teamcode.common.controller.ButtonListener;
import org.firstinspires.ftc.teamcode.common.enums.Color;
import org.firstinspires.ftc.teamcode.common.enums.Position;
import org.firstinspires.ftc.teamcode.common.menu.Menu;
import org.firstinspires.ftc.teamcode.common.menu.Screen;

import java.util.Arrays;
import java.util.List;

@Autonomous
public class Auto extends LinearOpMode {
    Robot robot;
    Processor processor;
    ElapsedTime elapsedTime;
    private static int stage;

    private final double heading = Math.toRadians(180);
    private final Pose farStart = new Pose(38, 8, heading);
    private final Pose closeStart = new Pose(24, 129, Math.toRadians(225));

    private final Pose farScore1Pose = new Pose(40, 8, heading);
    private final Pose farScore2Pose = new Pose(57, 25, heading);
    private final Pose farScore2Control = new Pose(60, 65, heading);
    private final Pose farScore3Pose = new Pose(50, 8, heading);
    private final Pose closeScore1Pose = new Pose(50, 84, heading);
    private final Pose closeScore2Pose = new Pose(55, 80, heading);


    private final Pose farPickup1Pose = new Pose(16, 8, heading);
    private final Pose farPickup2Pose = new Pose(13, 60, heading);
    private final Pose farPickup2Control = new Pose(60, 65, heading);
    private final Pose farPickup3Pose = new Pose(14, 36, heading);
    private final Pose farPickup3Control = new Pose(45, 35, heading);

    private final Pose farPickup4Pose = new Pose(24, 8, heading);
    private final Pose farPickup5Pose = new Pose(12, 8, heading);

    private final Pose closePickup1Pose = new Pose(23, 84, heading);
    private final Pose closePickup2Pose = new Pose(12, 55, heading);
    private final Pose closePickup2Control = new Pose(40, 50, heading);
    private final Pose closePickup3Pose = new Pose(10, 35, heading);
    private final Pose closePickup3Control = new Pose(50, 25, heading);


    private final Pose closeGateOpen = new Pose(8, 70, Math.toRadians(70));
    private final Pose closeGateOpenControl = new Pose(50, 70, heading);

    private final Pose farGateOpen = new Pose(8, 63, Math.toRadians(300));
    private final Pose farGateOpenControl = new Pose(50, 63, heading);


    private final Pose farParkPose = new Pose(30, 8, heading);
    private final Pose closeParkPose = new Pose(25, 70, Math.toRadians(270));


    private PathChain scorePreload, grabPickup1, scorePickup1, grabPickup2, openGate, scorePickup2, grabPickup3, scorePickup3, grabPickup4, scorePickup4, grabPickup5, scorePickup5, park;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        processor = new Processor();
        elapsedTime = new ElapsedTime();
        stage = 1;


        Menu menu = new Menu(new ButtonListener(gamepad1), telemetry);
        menu.addScreen(new Screen("Color", Arrays.asList(Color.values())))
                .addScreen(new Screen<>("Position", Arrays.asList(Position.values())));

        menu.runBlocking();

        List<Object> options = menu.getSelections();
        Robot.color = (Color) options.get(0);
        Robot.position = (Position) options.get(1);


        robot.lifter.liftDown();
        robot.limelight.setPipLine(Robot.color);
        robot.pins.preload();

        if (Robot.color == Color.BLUE && Robot.position == Position.Far) {

            robot.follower.setStartingPose(farStart);


            grabPickup1 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farStart, farPickup1Pose))
                    .setLinearHeadingInterpolation(farScore1Pose.getHeading(), farPickup1Pose.getHeading())
                    .setNoDeceleration()

                    .build();
            scorePickup1 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup1Pose, farScore2Pose))
                    .setLinearHeadingInterpolation(farPickup1Pose.getHeading(), farScore2Pose.getHeading())
                    .setBrakingStrength(4)
                    .setBrakingStart(2)
                    .build();
            grabPickup2 = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(farScore2Pose, farPickup2Control, farPickup2Pose))
                    .setLinearHeadingInterpolation(farScore2Pose.getHeading(), farPickup2Pose.getHeading())
                    .setNoDeceleration()
                    .build();
            openGate = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(farPickup2Pose, farGateOpenControl, farGateOpen))
                    .setLinearHeadingInterpolation(farPickup2Pose.getHeading(), farGateOpen.getHeading())
                    .setNoDeceleration()
                    .build();
            scorePickup2 = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(farGateOpen, farScore2Control, farScore2Pose))
                    .setLinearHeadingInterpolation(farGateOpen.getHeading(), farScore2Pose.getHeading())
                    .setBrakingStrength(4)
                    .build();
            grabPickup3 = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(farScore2Pose, farPickup3Control, farPickup3Pose))
                    .setLinearHeadingInterpolation(farScore2Pose.getHeading(), farPickup3Pose.getHeading())
                    .setNoDeceleration()
                    .build();

            scorePickup3 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup3Pose, farScore3Pose))
                    .setLinearHeadingInterpolation(farPickup3Pose.getHeading(), farScore3Pose.getHeading())
                    .setBrakingStrength(4)
                    .build();

            grabPickup4 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farScore3Pose, farPickup4Pose))
                    .setLinearHeadingInterpolation(farScore3Pose.getHeading(), farPickup4Pose.getHeading())
                    .setNoDeceleration()
                    .build();
            scorePickup4 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup4Pose, farScore1Pose))
                    .setLinearHeadingInterpolation(farPickup4Pose.getHeading(), farScore1Pose.getHeading())
                    .setBrakingStrength(4)
                    .build();

            grabPickup5 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farScore1Pose, farPickup5Pose))
                    .setLinearHeadingInterpolation(farScore1Pose.getHeading(), farPickup5Pose.getHeading())
                    .setNoDeceleration()
                    .build();
            scorePickup5 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup5Pose, farScore1Pose))
                    .setLinearHeadingInterpolation(farPickup5Pose.getHeading(), farScore1Pose.getHeading())
                    .setBrakingStrength(4)
                    .build();


            park = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farScore1Pose, farParkPose))
                    .setLinearHeadingInterpolation(farScore1Pose.getHeading(), farParkPose.getHeading())
                    .build();
        } else if (Robot.color == Color.RED && Robot.position == Position.Far) {

            robot.follower.setStartingPose(farStart.mirror());


            grabPickup1 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farStart.mirror(), farPickup1Pose.mirror()))
                    .setLinearHeadingInterpolation(farScore1Pose.mirror().getHeading(), farPickup1Pose.mirror().getHeading())
                    .setNoDeceleration()
                    .build();
            scorePickup1 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup1Pose.mirror(), farScore2Pose.mirror()))
                    .setLinearHeadingInterpolation(farPickup1Pose.mirror().getHeading(), farScore2Pose.mirror().getHeading())
                    .setBrakingStrength(4)
                    .build();
            grabPickup2 = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(farScore2Pose.mirror(), farPickup2Control.mirror(), farPickup2Pose.mirror()))
                    .setLinearHeadingInterpolation(farScore2Pose.mirror().getHeading(), farPickup2Pose.mirror().getHeading())
                    .setNoDeceleration()
                    .build();
            openGate = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(farPickup2Pose.mirror(), farGateOpenControl.mirror(), farGateOpen.mirror()))
                    .setLinearHeadingInterpolation(farPickup2Pose.mirror().getHeading(), farGateOpen.mirror().getHeading())
                    .setNoDeceleration()
                    .build();
            scorePickup2 = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(farGateOpen.mirror(), farScore2Control.mirror(), farScore2Pose.mirror()))
                    .setLinearHeadingInterpolation(farGateOpen.mirror().getHeading(), farScore2Pose.mirror().getHeading())
                    .setBrakingStrength(4)
                    .build();
            grabPickup3 = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(farScore2Pose.mirror(), farPickup3Control.mirror(), farPickup3Pose.mirror()))
                    .setLinearHeadingInterpolation(farScore2Pose.mirror().getHeading(), farPickup3Pose.mirror().getHeading())
                    .setNoDeceleration()
                    .build();

            scorePickup3 = robot.follower.pathBuilder()

                    .addPath(new BezierLine(farPickup3Pose.mirror(), farScore3Pose.mirror()))
                    .setLinearHeadingInterpolation(farPickup3Pose.mirror().getHeading(), farScore3Pose.mirror().getHeading())
                    .setBrakingStrength(4)
                    .build();
            grabPickup4 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farScore3Pose.mirror(), farPickup4Pose.mirror()))
                    .setLinearHeadingInterpolation(farScore3Pose.mirror().getHeading(), farPickup4Pose.mirror().getHeading())
                    .setNoDeceleration()
                    .build();
            scorePickup4 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup4Pose.mirror(), farScore1Pose.mirror()))
                    .setLinearHeadingInterpolation(farPickup4Pose.mirror().getHeading(), farScore1Pose.mirror().getHeading())
                    .setBrakingStrength(4)
                    .build();
            grabPickup5 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farScore1Pose.mirror(), farPickup5Pose.mirror()))
                    .setLinearHeadingInterpolation(farScore1Pose.mirror().getHeading(), farPickup5Pose.mirror().getHeading())
                    .setNoDeceleration()
                    .build();

            scorePickup5 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farPickup5Pose.mirror(), farScore1Pose.mirror()))
                    .setLinearHeadingInterpolation(farPickup5Pose.mirror().getHeading(), farScore1Pose.mirror().getHeading())
                    .setBrakingStrength(4)
                    .build();

            park = robot.follower.pathBuilder()
                    .addPath(new BezierLine(farScore1Pose.mirror(), farParkPose.mirror()))
                    .setLinearHeadingInterpolation(farScore1Pose.mirror().getHeading(), farParkPose.mirror().getHeading())
                    .setBrakingStart(4)
                    .build();
        } else if (Robot.color == Color.BLUE && Robot.position == Position.Close) {

            robot.follower.setStartingPose(closeStart);

            scorePreload = robot.follower.pathBuilder()
                    .addPath(new BezierLine(closeStart, closeScore1Pose))
                    .setLinearHeadingInterpolation(closeStart.getHeading(), closeScore1Pose.getHeading())
                    .setBrakingStrength(3)
                    .build();


            grabPickup1 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(closeScore1Pose, closePickup1Pose))
                    .setLinearHeadingInterpolation(closeScore1Pose.getHeading(), closePickup1Pose.getHeading())
                    .setNoDeceleration()

                    .build();

            openGate = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(closePickup1Pose, closeGateOpenControl, closeGateOpen))
                    .setLinearHeadingInterpolation(closePickup1Pose.getHeading(), closeGateOpen.getHeading())
                    .setNoDeceleration()
                    .build();

            scorePickup1 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(closeGateOpen, closeScore2Pose))
                    .setLinearHeadingInterpolation(closeGateOpen.getHeading(), closeScore2Pose.getHeading())
                    .setBrakingStrength(3)
                    .build();

            grabPickup2 = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(closeScore2Pose, closePickup2Control, closePickup2Pose))
                    .setLinearHeadingInterpolation(closeScore2Pose.getHeading(), closePickup2Pose.getHeading())
                    .setNoDeceleration()
                    .build();


            scorePickup2 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(closePickup2Pose, closeScore2Pose))
                    .setLinearHeadingInterpolation(closePickup2Pose.getHeading(), closeScore2Pose.getHeading())
                    .setBrakingStart(3)
                    .build();
            grabPickup3 = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(closeScore2Pose, closePickup3Control, closePickup3Pose))
                    .setLinearHeadingInterpolation(closeScore2Pose.getHeading(), closePickup3Pose.getHeading())
                    .setNoDeceleration()
                    .build();

            scorePickup3 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(closePickup3Pose, closeScore2Pose))
                    .setLinearHeadingInterpolation(closePickup3Pose.getHeading(), closeScore2Pose.getHeading())
                    .setBrakingStrength(3)
                    .build();


            park = robot.follower.pathBuilder()
                    .addPath(new BezierLine(closeScore2Pose, closeParkPose))
                    .setLinearHeadingInterpolation(closeScore2Pose.getHeading(), closeParkPose.getHeading())
                    .build();
        } else if (Robot.color == Color.RED && Robot.position == Position.Close) {

            robot.follower.setStartingPose(closeStart.mirror());

            scorePreload = robot.follower.pathBuilder()
                    .addPath(new BezierLine(closeStart.mirror(), closeScore1Pose.mirror()))
                    .setLinearHeadingInterpolation(closeStart.mirror().getHeading(), closeScore1Pose.mirror().getHeading())
                    .setBrakingStrength(3)
                    .build();


            grabPickup1 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(closeScore1Pose.mirror(), closePickup1Pose.mirror()))
                    .setLinearHeadingInterpolation(closeScore1Pose.mirror().getHeading(), closePickup1Pose.mirror().getHeading())
                    .setNoDeceleration()

                    .build();
            openGate = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(closePickup1Pose.mirror(), closeGateOpenControl.mirror(), closeGateOpen.mirror()))
                    .setLinearHeadingInterpolation(closePickup1Pose.mirror().getHeading(), closeGateOpen.mirror().getHeading())
                    .setNoDeceleration()
                    .build();
            scorePickup1 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(closeGateOpen.mirror(), closeScore2Pose.mirror()))
                    .setLinearHeadingInterpolation(closeGateOpen.mirror().getHeading(), closeScore2Pose.mirror().getHeading())
                    .setBrakingStrength(3)
                    .build();

            grabPickup2 = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(closeScore2Pose.mirror(), closePickup2Control.mirror(), closePickup2Pose.mirror()))
                    .setLinearHeadingInterpolation(closeScore2Pose.mirror().getHeading(), closePickup2Pose.mirror().getHeading())
                    .setNoDeceleration()
                    .build();


            scorePickup2 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(closePickup2Pose.mirror(), closeScore2Pose.mirror()))
                    .setLinearHeadingInterpolation(closePickup2Pose.mirror().getHeading(), closeScore2Pose.mirror().getHeading())
                    .setBrakingStart(3)
                    .build();
            grabPickup3 = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(closeScore2Pose.mirror(), closePickup3Control.mirror(130), closePickup3Pose.mirror()))
                    .setLinearHeadingInterpolation(closeScore2Pose.mirror().getHeading(), closePickup3Pose.mirror().getHeading())
                    .setNoDeceleration()
                    .build();

            scorePickup3 = robot.follower.pathBuilder()
                    .addPath(new BezierLine(closePickup3Pose.mirror(), closeScore2Pose.mirror()))
                    .setLinearHeadingInterpolation(closePickup3Pose.mirror().getHeading(), closeScore2Pose.mirror().getHeading())
                    .setBrakingStrength(3)
                    .build();


            park = robot.follower.pathBuilder()
                    .addPath(new BezierLine(closeScore2Pose.mirror(), closeParkPose.mirror()))
                    .setLinearHeadingInterpolation(closeScore2Pose.mirror().getHeading(), closeParkPose.mirror().getHeading())
                    .build();
        }
        robot.follower.update();

        waitForStart();


        robot.follower.setMaxPower(1);
        Robot.sort = false;


        while (!isStopRequested()) {
            if (Robot.position == Position.Far) {

                if (stage == 1) {

                    processor.override(new Shoot());
                    increment();
                } else if (stage == 2 && !processor.isBusy()) {
                    processor.override(new IntakeBall(true));
                    robot.follower.followPath(grabPickup1);
                    increment();
                } else if (stage == 3 && moveToNext()) {

                    processor.override(new IntakeTransfer());
                    robot.follower.followPath(scorePickup1);
                    increment();
                } else if (stage == 4 && !robot.follower.isBusy()) {
                    processor.override(new Shoot());
                    increment();
                } else if (stage == 5 && !processor.isBusy()) {
                    processor.override(new IntakeBall(false));
                    robot.follower.followPath(grabPickup2);
                    increment();
                } else if (stage == 6 && moveToNext()) {

                    robot.follower.followPath(openGate, 1, true);
                    elapsedTime.reset();
                    increment();
                    stage = 100;
                } else if (stage == 100 && robot.follower.atParametricEnd()) {
                    if (robot.ballsInIntake()) {
                        processor.override(new IntakeTransfer());
                    }
                    if (elapsedTime.milliseconds()>1500){
                        robot.follower.followPath(scorePickup2);
                        stage = 7;
                    }

                } else if (stage == 7 && !robot.follower.isBusy()) {
                    processor.override(new Shoot());
                    increment();
                } else if (stage == 8 && !processor.isBusy()) {
                    processor.override(new IntakeBall(true));
                    robot.follower.followPath(grabPickup3);

                    increment();

                } else if (stage == 9 && moveToNext()) {
                    if (robot.ballsInIntake()) {
                        processor.override(new IntakeTransfer());
                        increment();
                    } else if (elapsedTime.milliseconds() > 500) {
                        increment();
                    }

                    robot.follower.followPath(scorePickup3);


                } else if (stage == 10 && !robot.follower.isBusy()) {
                    processor.override(new Shoot());
                    increment();
                } else if (stage == 11 && !processor.isBusy()) {
                    processor.override(new IntakeBall(true));
                    robot.follower.followPath(grabPickup4);
                    increment();
                } else if (stage == 12 && moveToNext()) {
                    processor.override(new IntakeTransfer());
                    robot.follower.followPath(scorePickup4);
                    increment();
                } else if (stage == 13 && !robot.follower.isBusy()) {
                    processor.override(new Shoot());
                    increment();
                } else if (stage == 14 && !processor.isBusy()) {
                    processor.override(new IntakeBall(true));
                    robot.follower.followPath(grabPickup5);
                    increment();
                } else if (stage == 15 && moveToNext()) {

                    processor.override(new IntakeTransfer());
                    robot.follower.followPath(scorePickup5);
                    increment();
                } else if (stage == 16 && !robot.follower.isBusy()) {
                    processor.override(new Shoot());
                    increment();
                } else if (stage == 17 && !processor.isBusy()) {
                    robot.follower.followPath(park);

                    increment();
                } else if (stage == 18) {
                    robot.limelight.setPipLine(null);
                    if (Robot.color == Color.RED) {
                        robot.turret.setTargetAngle(-60);
                    } else if (Robot.color == Color.BLUE) {
                        robot.turret.setTargetAngle(60);
                    }
                    if (robot.limelight.getMotif() != 0) {
                        Robot.motif = robot.limelight.getMotif();
                    }
                }
            } else if (Robot.position == Position.Close) {

                if (stage == 1) {
                    processor.override(new IntakeBall(false));
                    robot.follower.followPath(scorePreload);
                    increment();
                } else if (stage == 2 && !robot.follower.isBusy()) {
                    processor.override(new Shoot());
                    increment();
                } else if (stage == 3 && !processor.isBusy()) {
                    processor.override(new IntakeBall(false));
                    robot.follower.followPath(grabPickup1);
                    increment();
                } else if (stage == 4 && moveToNext()) {
                    robot.follower.followPath(openGate);
                    increment();

                } else if (stage == 5 && robot.follower.atParametricEnd()) {
                    if (elapsedTime.milliseconds()>500){
                        robot.follower.followPath(scorePickup1);
                        increment();
                    }


                } else if (stage == 6) {
                    if (robot.ballsInIntake()) {
                        processor.override(new IntakeTransfer());
                        increment();
                    } else if (elapsedTime.milliseconds() > 500) {
                        increment();
                    }

                } else if (stage == 7 && !robot.follower.isBusy()) {
                    processor.override(new Shoot());
                    increment();



                } else if (stage == 8 && !processor.isBusy()) {
                    processor.override(new IntakeBall(false));
                    robot.follower.followPath(grabPickup2);
                    increment();


                } else if (stage == 9 && moveToNext()) {
                    robot.follower.followPath(scorePickup2);
                    increment();
                } else if (stage == 10) {
                    if (robot.ballsInIntake()) {
                        processor.override(new IntakeTransfer());
                        increment();
                    } else if (elapsedTime.milliseconds() > 1000) {
                        increment();
                    }
                } else if (stage == 11 && !robot.follower.isBusy()) {
                    processor.override(new Shoot());
                    increment();

                } else if (stage == 12 && !processor.isBusy()) {
                    processor.override(new IntakeBall(false));
                    robot.follower.followPath(grabPickup3);
                    increment();
                } else if (stage == 13 && moveToNext()) {
                    robot.follower.followPath(scorePickup3);
                    increment();
                } else if (stage == 14) {
                    if (robot.ballsInIntake()) {
                        processor.override(new IntakeTransfer());
                        increment();
                    } else if (elapsedTime.milliseconds() > 1000) {
                        increment();
                    }
                } else if (stage == 15 && !robot.follower.isBusy()) {
                    processor.override(new Shoot());
                    increment();
                } else if (stage == 16 && !processor.isBusy()) {
                    robot.follower.followPath(park);
                    robot.limelight.setPipLine(null);
                    if (Robot.color == Color.RED) {
                        robot.turret.setTargetAngle(40);
                    } else if (Robot.color == Color.BLUE) {
                        robot.turret.setTargetAngle(-40);
                    }
                    increment();
                } else if (stage == 17) {
                    if (robot.limelight.getMotif() != 0) {
                        Robot.motif = robot.limelight.getMotif();
                    }
                }

            }
            telemetry.addData("stage", stage);
            telemetry.addData("motif", robot.limelight.getMotif());
            telemetry.update();
            robot.update();
            processor.update(robot);
        }


    }

    private void increment() {
        stage++;
        elapsedTime.reset();
    }

    private boolean moveToNext() {
        return robot.follower.atParametricEnd() || Robot.threeBalls;
    }
}



