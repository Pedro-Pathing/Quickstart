package org.firstinspires.ftc.teamcode.Auto; // make sure this aligns with class location

import static org.firstinspires.ftc.teamcode.Mechanics.Robot.autoEnd;
import static org.firstinspires.ftc.teamcode.Mechanics.Robot.intake;
import static org.firstinspires.ftc.teamcode.Mechanics.Robot.spindexer;
import static org.firstinspires.ftc.teamcode.Paths.PathsImproved.blueFar;
import static org.firstinspires.ftc.teamcode.Paths.PathsImproved.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Mechanics.Color;
import org.firstinspires.ftc.teamcode.Mechanics.Robot;
import org.firstinspires.ftc.teamcode.Mechanics.Shooter;
import org.firstinspires.ftc.teamcode.Mechanics.Spind;
import org.firstinspires.ftc.teamcode.Mechanics.Turret;
import org.firstinspires.ftc.teamcode.Mechanics.Vision;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "blue far auto 🔥", group = "Red auto")
public class AutonomousBlueFar extends OpMode {
    private Follower follower;
    private double shotPower;
    private final double sPow = 1;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    String motif = "";
    public Vision camera = new Vision();
    private int pathState;
    private final Pose startPose = new Pose(60, 9, Math.PI/2);

    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 1:
                follower.followPath(scoreP, true);
                setPathState(2);
                break;
            case 2:
                if (!follower.isBusy()) {
                    shotPower = sPow;
                    if (pathTimer.getElapsedTimeSeconds() > 3 && Spind.Launch3Balls(pathTimer, 0.75,1)) {
                        setPathState(3);
                        shotPower = 0;
                    }
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(intake1, true);
                    if (Spind.intaking(pathTimer,1) || pathTimer.getElapsedTimeSeconds() > 6)
                        setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    follower.followPath(gate, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    Robot.intake.setPower(-1);
                    follower.followPath(score1, true);
                    setPathState(7);
                    shotPower = sPow;
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    setPathState(8);
                }
                break;
            case 8:
                if(Spind.Launch3Balls(pathTimer, 0.75,1) || pathTimer.getElapsedTimeSeconds() > 4){
                    Robot.intake.setPower(0);
                    follower.followPath(intake2, true);
                    shotPower = 0;
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(intake2, true);
                    if (Spind.intaking(pathTimer, 1) || pathTimer.getElapsedTimeSeconds() > 6) {
                        setPathState(10);
                        shotPower = sPow;
                    }
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    Robot.intake.setPower(-1);
                    follower.followPath(score2, true);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    setPathState(12);
                }
                break;
            case 12:
                Robot.intake.setPower(0);
                if(Spind.Launch3Balls(pathTimer, 0.75,1) || pathTimer.getElapsedTimeSeconds() > 4){
                    follower.followPath(intake3, true);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    shotPower = 0;
                    follower.followPath(intake3, true);

                    if (Spind.intaking(pathTimer, 1) || pathTimer.getElapsedTimeSeconds() > 6) {
                        setPathState(14);
                        shotPower = sPow;
                    }
                }
                break;
            case 14:
                if(!follower.isBusy()) {
                    Robot.intake.setPower(-1);
                    follower.followPath(score3,true);
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    setPathState(16);
                }
                break;
            case 16:
                Robot.intake.setPower(0);
                if(Spind.Launch3Balls(pathTimer, 0.75,1) || pathTimer.getElapsedTimeSeconds() > 4){
                    follower.followPath(end, true);
                    shotPower = 0;
                    setPathState(17);
                }
                break;
            case 17:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;

        }
    }
    public void setPathState(int pState){
        pathState=pState;
        pathTimer.resetTimer();
    }
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        if(motif.isEmpty()){
            motif = camera.findMotif();
        }
        Shooter.setPower(shotPower);
        Shooter.autoShotHood(follower.getPose().getX(), 144 - follower.getPose().getY(), follower.getHeading(), false);
        Turret.faceGoal(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading(), false, 0);
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        Robot.init(hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        blueFar(follower);
        follower.setStartingPose(startPose);
        camera.initAprilTag(hardwareMap);
        if (USE_WEBCAM) {
            try {
                camera.setManualExposure(6, 250, telemetry);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        //this gets the motif, however it could be inconsistent, so i'll try to make it so the camera points at the thing until it gets the motif and then it can aim towards the goal.
        motif = camera.findMotif();
        telemetry.addData("motif",motif);
        telemetry.update();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(1);
        if (motif.isEmpty())
            motif = "PPG";
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        autoEnd = follower.getPose();
    }

}