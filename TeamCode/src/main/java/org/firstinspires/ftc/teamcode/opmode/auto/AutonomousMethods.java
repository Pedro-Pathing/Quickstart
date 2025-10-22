package org.firstinspires.ftc.teamcode.opmode.auto;

import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.Path;

public class AutonomousMethods {

    public static Path buildPath(Pose pose1, Pose pose2, double time) {
        Path path = new Path(new BezierLine(pose1, pose2));
        path.setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading());
        return path;
    }


    public static Path buildPathTurnLater(Pose pose1, Pose pose2, double time) {
        Path path = new Path(new BezierLine(pose2, pose1));
        path.setLinearHeadingInterpolation(pose2.getHeading(), pose1.getHeading());
        return path;
    }

    public static Path buildPath(Pose pose1, Pose pose2) {
        return buildPath(pose1, pose2, 1);
    }


    public static Path buildCurve(Pose pose1, Pose pose2, Pose control1, double time) {
        Path path = new Path(new BezierCurve(pose1, control1, pose2));
        path.setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading());
        return path;
    }


    public static Path buildCurveTurnLater(Pose pose1, Pose pose2, Pose control1, double time) {
        Path path = new Path(new BezierCurve(pose2, control1, pose1));
        path.setLinearHeadingInterpolation(pose2.getHeading(), pose1.getHeading());
        return path;
    }

    public static Path buildCurve(Pose pose1, Pose pose2, Pose control1) {
        return buildCurve(pose1, pose2, control1, 1);
    }


    public static Path buildCurve(Pose pose1, Pose pose2, Pose control1, Pose control2, double time) {
        Path path = new Path(new BezierCurve(pose1, control1, control2, pose2));
        path.setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading());
        return path;
    }


    public static Path buildCurveTurnLater(Pose pose1, Pose pose2, Pose control1, Pose control2, double time) {
        Path path = new Path(new BezierCurve(pose2, control2, control1, pose1));
        path.setLinearHeadingInterpolation(pose2.getHeading(), pose1.getHeading());
        return path;
    }

    public static Path buildCurve(Pose pose1, Pose pose2, Pose control1, Pose control2) {
        return buildCurve(pose1, pose2, control1, control2, 1);
    }
}
