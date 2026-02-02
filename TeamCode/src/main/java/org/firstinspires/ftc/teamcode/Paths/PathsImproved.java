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
    public static PathChain scoreP, intake1, gate, score1, intake2, score2, intake3, score3, end;

    public static void blueFar(Follower follower) {
        scoreP = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(60.000, 9.000), new Pose(60.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        intake1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(63.000, 84.000),
                                new Pose(20.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        gate = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(20.000, 84.000),
                                new Pose(24, 76),
                                new Pose(18.000, 72.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        score1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(20.000, 84.000),
                                new Pose(63, 84.000),
                                new Pose(60, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        intake2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(61.657, 56.221),
                                new Pose(15.000, 56.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        score2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(15.000, 56.000),
                                new Pose(61.866, 56.012),
                                new Pose(60.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        intake3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(59.985, 30.514),
                                new Pose(15.000, 36.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        score3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(15.000, 36.000),
                                new Pose(59.985, 30.514),
                                new Pose(60.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        end = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(60, 84),
                                new Pose(36,84)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();
    }

    public static void redFar(Follower follower) {
        scoreP = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-60.000, 9.000), new Pose(144-60.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        intake1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-60.000, 84.000),
                                new Pose(144-63.000, 84.000),
                                new Pose(144-20.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        gate = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-20.000, 84.000),
                                new Pose(144-24, 76),
                                new Pose(144-18.000, 72.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        score1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-20.000, 84.000),
                                new Pose(144-63, 84.000),
                                new Pose(144-60, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();

        intake2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-60.000, 84.000),
                                new Pose(144-61.657, 56.221),
                                new Pose(144-15.000, 56.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        score2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-15.000, 56.000),
                                new Pose(144-61.866, 56.012),
                                new Pose(144-60.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();

        intake3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-60.000, 84.000),
                                new Pose(144-59.985, 30.514),
                                new Pose(144-15.000, 36.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        score3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-15.000, 36.000),
                                new Pose(144-59.985, 30.514),
                                new Pose(144-60.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();

        end = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(144-60, 84),
                                new Pose(144-36,84)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();
    }


    public static void bluePartner(Follower follower) {
        scoreP = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60.000, 9.000),

                                new Pose(60.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))

                .build();

        intake1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(61.657, 56.221),
                                new Pose(15.000, 56.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        gate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.000, 56.000),
                                new Pose(31.577, 66.881),
                                new Pose(18.000, 67.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        score1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(18.000, 67.000),

                                new Pose(60.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))

                .build();

        intake2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(59.985, 30.514),
                                new Pose(15.000, 36.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        score2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.000, 36.000),
                                new Pose(59.985, 30.514),
                                new Pose(60.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))

                .build();

        end = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(60, 84),
                                new Pose(36,84)
                        )
                )
                .setLinearHeadingInterpolation(Math.PI/2, Math.PI)
                .build();
    }

    public static void redPartner(Follower follower) {
        scoreP = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(144-60.000, 9.000),

                                new Pose(144-60.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))

                .build();

        intake1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(144-60.000, 84.000),
                                new Pose(144-61.657, 56.221),
                                new Pose(144-15.000, 56.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        gate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(144-15.000, 56.000),
                                new Pose(144-31.577, 66.881),
                                new Pose(144-18.000, 67.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        score1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(144-18.000, 67.000),

                                new Pose(144-60.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                .build();

        intake2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(144-60.000, 84.000),
                                new Pose(144-59.985, 30.514),
                                new Pose(144-15.000, 36.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        score2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(144-15.000, 36.000),
                                new Pose(144-59.985, 30.514),
                                new Pose(144-60.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                .build();

        end = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(144-60, 84),
                                new Pose(144-36,84)
                        )
                )
                .setLinearHeadingInterpolation(Math.PI/2, Math.PI)
                .build();
    }

}
