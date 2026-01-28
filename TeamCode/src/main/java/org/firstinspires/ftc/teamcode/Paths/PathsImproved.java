package org.firstinspires.ftc.teamcode.Paths;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
public class PathsImproved {
    public PathChain scoreP;
    public PathChain intake1;
    public PathChain gate;
    public PathChain score1;
    public PathChain intake2;
    public PathChain score2;
    public PathChain intake3;
    public PathChain score3;

    public PathsImproved(Follower follower) {
        scoreP = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(60.000, 9.000), new Pose(60.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        intake1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(60.000, 84.000), new Pose(16.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        gate = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(16.000, 84.000),
                                new Pose(22.156, 76.075),
                                new Pose(15.000, 72.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        score1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(15.000, 72.000),
                                new Pose(59.985, 69.388),
                                new Pose(60.000, 84.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        intake2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(61.657, 56.221),
                                new Pose(12.000, 60.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        score2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(12.000, 60.000),
                                new Pose(61.866, 56.012),
                                new Pose(60.000, 84.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        intake3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(59.985, 30.514),
                                new Pose(16.000, 36.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        score3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(16.000, 36.000),
                                new Pose(59.985, 30.514),
                                new Pose(60.000, 84.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
    }
}
