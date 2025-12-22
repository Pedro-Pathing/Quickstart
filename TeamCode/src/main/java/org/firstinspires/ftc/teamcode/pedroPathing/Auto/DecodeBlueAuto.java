package org.firstinspires.ftc.teamcode.pedroPathing.Auto;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class DecodeBlueAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        //Start Position -End Position
        //drive > Movement
        //Shoot >Attempts to Score the Artifact

        DRIVE_STARTPOSITIONTOSHOOT,
        PICKUP_RANGEE1Blue,
        avalerballeRangee1,

        PICKUP_RANGEE2Blue,
        PICKUP_RANGEE3Blue,
        SHOOT_PRELOAD

    }

    PathState pathState;

    private final Pose startPose = new Pose(33.9416569,135.598599, Math.toRadians(180));
    private final Pose shootPose = new Pose(42.00700116,103.0011668,Math.toRadians(137));

    private final Pose PickUpBall1 = new Pose (48.3920653, 82.8378063, Math.toRadians(180));

    private final Pose avalerballeRangee1 = new Pose (13.61026837, 84.1820303, Math.toRadians(180));
    private PathChain driveStartoShootPos, driveShoot2pickup1Pos, driveAvalerpremiereLigne;

    public void buildPaths() {
        //put the coordinate from start to shooting
        driveStartoShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        //du premiershoot au pickupnuméro 1
        driveShoot2pickup1Pos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, PickUpBall1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), PickUpBall1.getHeading())
                .build();

        //de la l'alignement pickup1 à la derniere balle rangée 1
        driveAvalerpremiereLigne = follower.pathBuilder()
                .addPath(new BezierLine(PickUpBall1,avalerballeRangee1 ))
                .setLinearHeadingInterpolation(PickUpBall1.getHeading(), avalerballeRangee1.getHeading())
                .setVelocityConstraint(0.25)
                .build();


    }
    public void statePathUpdate(){
        switch(pathState) {
            case DRIVE_STARTPOSITIONTOSHOOT:
                follower.followPath(driveStartoShootPos, true); //true will hold the positon
                setPathState(PathState.SHOOT_PRELOAD); // Reset Timer + make new staet
                break;

            case SHOOT_PRELOAD:
                ;
                // check is follow done is path
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>5){
                    // TODO add ShooterLogic 3 tirs
                    telemetry.addLine("Shooting");
                    // transition to next state
                    // ce
                    follower.followPath(driveShoot2pickup1Pos, true);
                    setPathState(PathState.PICKUP_RANGEE1Blue);
                }

                break;

            case PICKUP_RANGEE1Blue:
                // check is follow done is path
                if (!follower.isBusy()) {
                    // TODO add ShooterLogic 3 tirs
                    telemetry.addLine("Done with Shooting 1, deplacement vers premiere rangée");
                    // transition to next state
                    follower.followPath(driveAvalerpremiereLigne, true);
                    setPathState(PathState.avalerballeRangee1);
                }
                break;

            case avalerballeRangee1:
                // check is follow done is path
                if (!follower.isBusy()) {
                    // TO DO demarer intake , tourner indexeur des dectetion balles)
                    telemetry.addLine("ramassage terminé");
                    // transition to next state
                }
                break;


            default:
                telemetry.addLine("No Statement Commanded");
                break;
        }

    }

    public void setPathState (PathState newState){
        pathState = newState;
        pathTimer.resetTimer();

    }

    @Override
    public void init () {
        pathState = PathState.DRIVE_STARTPOSITIONTOSHOOT;
        pathTimer=new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setPose(startPose);

    }

    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);

    }
    @Override
    public void loop (){
        follower.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x",follower.getPose().getX());
        telemetry.addData("y",follower.getPose().getY());
        telemetry.addData("heading",follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());

    }
}
