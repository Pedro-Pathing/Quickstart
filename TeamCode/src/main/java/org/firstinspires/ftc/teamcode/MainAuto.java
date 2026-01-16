package org.firstinspires.ftc.teamcode;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Storage;
import org.firstinspires.ftc.teamcode.subsystems.Transitions;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous
public class MainAuto extends NextFTCOpMode {
    {
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
                )
        );
    }

    public static Pose endPose;



    public static final Pose startPoseFarBlue = new Pose(56, 8, Math.toRadians(270)); // Start Pose of our robot.
    public static final Pose scorePoseCloseBlue = new Pose(20, 123, Math.toRadians(323)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public static final Pose scorePoseBlue = new Pose(68, 76, Math.toRadians(315)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    public static final Pose intakeAlign1Blue = new Pose(68, 84, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public static final Pose intake1Blue = new Pose(16, 84, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.


    public static final Pose startPoseFarRed = new Pose(87, 8, Math.toRadians(270)); // Start Pose of our robot.
    public static final Pose startPoseCloseRed = new Pose(124, 123, Math.toRadians(37)); // Start Pose of our robot.
    public static final Pose scorePoseRed = new Pose(76, 76, Math.toRadians(225)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.


    private final Pose startPose = startPoseFarBlue;
    public static final Pose scorePose = scorePoseBlue;
    //public static final Pose scorePosebutActually = new Pose(73, 70, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.


    Path scorePreload = new Path(new BezierLine(startPose, scorePose));
    Path intakeAlign1 = new Path(new BezierLine(scorePose, intakeAlign1Blue));
    Path intake1 = new Path(new BezierLine(intakeAlign1Blue, intake1Blue));
    Path intakeOut1 = new Path(new BezierLine(intake1Blue, intakeAlign1Blue));
    Path score1 = new Path(new BezierLine(intakeAlign1Blue, scorePose));





    private Command autonomousRoutine() {
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        intakeAlign1.setLinearHeadingInterpolation(scorePose.getHeading(), intakeAlign1Blue.getHeading());
        intake1.setLinearHeadingInterpolation(intakeAlign1Blue.getHeading(), intake1Blue.getHeading());
        score1.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        int standardDelay = 1;

        return new SequentialGroup(
                new FollowPath(scorePreload),
                new Delay(standardDelay),
                Robot.outtakeAll,
                new Delay(standardDelay),
                new FollowPath(intakeAlign1),
                new Delay(standardDelay),
                // Start running intake procedure
                new FollowPath(intake1),
                new Delay(standardDelay),
                new FollowPath(intakeOut1),
                new Delay(standardDelay),
                new FollowPath(score1),
                Robot.outtakeAll
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