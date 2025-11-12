package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.util.Timer;


/**
 * Standard Robot TeleOp for FTC using Pedro Pathing.
 * Handles robot driving, shooter motor control, and turret control (CRServo).
 *
 * Author:
 *   Baron Henderson – 20077 The Indubitables (modified by Kushal Madhabhaktula)
 * Version:
 *   3.1, 10/2025
 */
@TeleOp(name = "RobotTeleop", group = "Examples")
public class RobotTeleop extends OpMode {
    private Timer timer;
    private Follower follower;
    private Robot robot;
    private static final double DEAD_ZONE = 0.1;
    private static final double TURRET_DEADZONE = 0.3; // Tighter alignment threshold

    private final Pose startPose = new Pose(0, 0, 0);

    private Pose currentPose = new Pose(0,0,0);

    private boolean is_RapidFireOn = false;
    private boolean targetTracking_enabled = true;

    @Override
    public void init() {
        timer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();

        robot = new Robot(hardwareMap);
        telemetry.addLine("RobotTeleop Initialized (CRServo turret)");
        telemetry.update();
    }

    @Override
    public void start() {
        timer.resetTimer();
        follower.startTeleopDrive();
        follower.setMaxPower(1.0);
    }

    private boolean is_CloseShot() {
        return gamepad2.b;
    }

    private boolean is_FarShot() {
        return gamepad2.x;
    }

    private boolean is_Intaking() {
        return gamepad2.left_bumper;
    }

    private boolean is_Shooting() {
        return gamepad2.dpad_up;
    }

    private boolean is_HumanPlayer() {
        return gamepad1.a;
    }

    private boolean is_FlywheelOff() {
        return gamepad2.y;
    }

//    private boolean is_TurretLeft() {
//        return gamepad2.dpad_left;
//    }
//
//    private boolean is_TurretRight() {
//        return gamepad2.dpad_right;
//    }

    private boolean is_MidRangeShot() {
        return gamepad2.a;
    }

    private boolean is_ShootingRapidFire() {
        return gamepad2.dpad_down;
    }



    @Override
    public void loop() {

        double xInput = Math.abs(gamepad1.left_stick_x) > DEAD_ZONE ? -gamepad1.left_stick_x : 0;
        double yInput = Math.abs(gamepad1.left_stick_y) > DEAD_ZONE ? -gamepad1.left_stick_y : 0;
        // NOTE: rotation is negated to match PedroPathing's TeleOp example (prevents reversed/odd rotation behavior)
        double turnInput = Math.abs(gamepad1.right_stick_x) > DEAD_ZONE ? -gamepad1.right_stick_x : 0;

        double powerScale = gamepad1.right_trigger > 0.5 ? 0.25 : 1.0;

        follower.updateErrors();
        follower.updateVectors();
        follower.setTeleOpDrive(
                yInput * powerScale,  // forward/backward
                xInput * powerScale,  // strafe
                turnInput * powerScale, // rotation (negated)
                true                   // robot-centric
        );

        follower.update();

        if (targetTracking_enabled) {
            robot.vision.update();
        }



        if (is_CloseShot()) {
            robot.shooter.startCloseShoot();
        } else if (is_FarShot()) {
            robot.shooter.startFarShoot();
        } else if (is_MidRangeShot()) {
            robot.shooter.startMidShoot();
        } else if (is_HumanPlayer()) {
            robot.shooter.startHumanIntake();
        } else if (is_FlywheelOff()){
            robot.shooter.stopShoot();
        }

         if (is_Intaking()) {
            robot.intake.startIntakeOnly();
        } else {
            robot.intake.stopIntake();
         }

          if (is_Shooting()) {
              if (robot.shooter.reachedSpeed()) {
                  if (is_RapidFireOn) {
                      robot.intake.startIntakeAndTransfer();
                  } else {
                      robot.intake.startTransferOnly();
                  }
                  gamepad1.rumble(1000);
                  gamepad2.rumble(1000);
              }
          } else {
            robot.intake.stopTransfer();
          }

        if (is_ShootingRapidFire() && timer.getElapsedTime() > 500) {
                is_RapidFireOn = !is_RapidFireOn;
        }



        // Turret control (fixed: check gamepad2 on both dpad sides)
        if (gamepad2.dpad_right && !gamepad2.dpad_left) {
            robot.turret.goRight(); // rotate right
        } else if (gamepad2.dpad_left && !gamepad2.dpad_right) {
            robot.turret.goLeft(); // rotate left
        } else {
            robot.turret.stopTurret();
        }

        currentPose = follower.getPose();
        robot.shooter.shooterLightUpdate();
        telemetry.addData("Rapid Fire On: ", is_RapidFireOn);
        telemetry.addData("Drive X", xInput);
        telemetry.addData("Drive Y", yInput);
        telemetry.addData("Turn", turnInput);
        //telemetry.addData("Turret Power", turretCR.getPower());
        telemetry.addData("Pose X", follower.getPose().getX());
        telemetry.addData("Pose Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.shooter.stopShoot();
        robot.intake.stopIntake();
        robot.intake.stopTransfer();
        //if (turretCR != null) turretCR.setPower(0.0);
    }
}
