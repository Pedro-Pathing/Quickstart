package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Storage;
import org.firstinspires.ftc.teamcode.subsystems.Transitions;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.Command;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;



@Autonomous(name = "TheGoodAuto Use This One")
public class TheGoodAutoBlue extends NextFTCOpMode {



    public TheGoodAutoBlue() {
        addComponents(
                BulkReadComponent.INSTANCE, // TODO: make actual MANUAL mode bulkreading (we don't need to also read the expansion hub every loop)
                BindingsComponent.INSTANCE,
                CommandManager.INSTANCE,
                new SubsystemComponent(
                        Storage.INSTANCE,
                        Robot.INSTANCE,
                        Drive.INSTANCE,
                        Intake.INSTANCE,
                        Outtake.INSTANCE,
                        Transitions.INSTANCE
                ),
                new PedroComponent(Constants::createFollower)

        );
    }

    public static final Pose startPoseBlue = new Pose(56, 8, Math.toRadians(270)); // Start Pose of our robot.
    public static final Pose scorePoseBlue = new Pose(73, 70, Math.toRadians(315)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    public static final Pose startPoseRed = new Pose(87, 8, Math.toRadians(270)); // Start Pose of our robot.
    public static final Pose scorePoseRed = new Pose(73, 70, Math.toRadians(225)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    public static Pose startPose; // Start Pose of our robot.
    public static Pose scorePose; // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.


    public static final Pose scorePosebutActually = new Pose(73, 70, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.


    public static Pose endPose;



    Path scorePreload = new Path(new BezierLine(startPose, scorePose));


    private Command autonomousRoutine() {
        if(Drive.currentAlliance == Robot.Alliance.RED){
            startPose = startPoseRed;
            scorePose = startPoseRed;

        }
        else {
            startPose = startPoseBlue;
            scorePose = startPoseBlue;
        }

        endPose = startPose;

        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        return new SequentialGroup(
                new FollowPath(scorePreload),
                new Delay(2),
                Robot.INSTANCE.outtakeAll
        );
    }

    @Override
    public void onStartButtonPressed() {
        follower().setStartingPose(startPose);
        autonomousRoutine().schedule();
        follower().breakFollowing();
    }

    public void onUpdate(){
        Drive.telemetryM.update();
        follower().update();
    }

    public void onStop(){
        endPose = follower().getPose();
    }



}