package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Drivetrain_;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass;

public class Pedro_Paths {
    // The variable to store PedroPathing's follower object for path building
    public Follower follower;

    /**
     * Define primary poses to be used in paths
      */
    // Start Poses
    public Pose startPose1 = new Pose(112, 136.6, Math.toRadians(270));
    public Pose startPose2 = new Pose(88, 7.4, Math.toRadians(90));

    // Define all path variables
    public PathChain startPose1_to_preload1, startPose1_to_preload2, startPose1_to_preload3;
    public PathChain startPose2_to_preload1, startPose2_to_preload2, startPose2_to_preload3;

    public Pose autoMirror(Pose pose, LoadHardwareClass.Alliance alliance){
        int mult = 1;
        if (alliance == LoadHardwareClass.Alliance.BLUE){
            mult = -1;
        }
        return new Pose(
                144 - pose.getX(),
                144 - pose.getY(),
                Math.atan2(Math.sin(pose.getHeading()), mult * Math.cos(pose.getHeading()))
        );
    }

    /**
     * Builds all the paths, mirroring them to the other side of the field if necessary
     */
    public void buildPaths(LoadHardwareClass.Alliance alliance){
        // All paths are for the RED side of the field. they will be mirrored if necessary.

        // Start Pose 1 to Preloads
        startPose1_to_preload3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(startPose1, alliance),
                        autoMirror(new Pose(89.000, 136.600), alliance),
                        autoMirror(new Pose(89.000, 30.000), alliance),
                        autoMirror(new Pose(95.000, 35.500), alliance),
                        autoMirror(new Pose(110.000, 35.500), alliance)
                ))
                .setLinearHeadingInterpolation(startPose1.getHeading(), Math.toRadians(0))
                .addPath(new BezierLine(
                        autoMirror(new Pose(110.000, 35.500), alliance),
                        autoMirror(new Pose(132.000, 35.500), alliance)
                ))
                .setTangentHeadingInterpolation()
                .build();
        startPose1_to_preload2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(startPose1, alliance),
                        autoMirror(new Pose(89.000, 136.600), alliance),
                        autoMirror(new Pose(89.000, 54.000), alliance),
                        autoMirror(new Pose(95.000, 59.500), alliance),
                        autoMirror(new Pose(110.000, 59.500), alliance)
                ))
                .setLinearHeadingInterpolation(startPose1.getHeading(), Math.toRadians(0))
                .addPath(new BezierLine(
                        autoMirror(new Pose(110.000, 59.500), alliance),
                        autoMirror(new Pose(132.000, 59.500), alliance)
                ))
                .setTangentHeadingInterpolation()
                .build();
        startPose1_to_preload1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(startPose1, alliance),
                        autoMirror(new Pose(89.000, 136.600), alliance),
                        autoMirror(new Pose(89.000, 78.000), alliance),
                        autoMirror(new Pose(95.000, 83.500), alliance),
                        autoMirror(new Pose(110.000, 83.500), alliance)
                ))
                .setLinearHeadingInterpolation(startPose1.getHeading(), Math.toRadians(0))
                .addPath(new BezierLine(
                        autoMirror(new Pose(110.000, 83.500), alliance),
                        autoMirror(new Pose(132.000, 83.500), alliance)
                ))
                .setTangentHeadingInterpolation()
                .build();

        // Start Pose 2 to Preloads
        startPose2_to_preload3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(startPose2, alliance),
                        autoMirror(new Pose(89.000, 136.600), alliance),
                        autoMirror(new Pose(89.000, 41.000), alliance),
                        autoMirror(new Pose(95.000, 35.500), alliance),
                        autoMirror(new Pose(110.000, 35.500), alliance)
                ))
                .setLinearHeadingInterpolation(startPose2.getHeading(), Math.toRadians(0))
                .addPath(new BezierLine(
                        autoMirror(new Pose(110.000, 35.500), alliance),
                        autoMirror(new Pose(132.000, 35.500), alliance)
                ))
                .setTangentHeadingInterpolation()
                .build();
        startPose2_to_preload2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(startPose2, alliance),
                        autoMirror(new Pose(89.000, 136.600), alliance),
                        autoMirror(new Pose(89.000, 65.000), alliance),
                        autoMirror(new Pose(95.000, 59.500), alliance),
                        autoMirror(new Pose(110.000, 59.500), alliance)
                ))
                .setLinearHeadingInterpolation(startPose1.getHeading(), Math.toRadians(0))
                .addPath(new BezierLine(
                        autoMirror(new Pose(110.000, 59.500), alliance),
                        autoMirror(new Pose(132.000, 59.500), alliance)
                ))
                .setTangentHeadingInterpolation()
                .build();
        startPose2_to_preload1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(startPose2, alliance),
                        autoMirror(new Pose(89.000, 89.000), alliance),
                        autoMirror(new Pose(95.000, 83.500), alliance),
                        autoMirror(new Pose(110.000, 83.500), alliance)
                ))
                .setLinearHeadingInterpolation(startPose2.getHeading(), Math.toRadians(0))
                .addPath(new BezierLine(
                        autoMirror(new Pose(110.000, 83.500), alliance),
                        autoMirror(new Pose(132.000, 83.500), alliance)
                ))
                .setTangentHeadingInterpolation()
                .build();
    }

    /**
     * Must be called after MecanumDrivetrainClass is initialized.
     * @param follow PedroPathing's follower object, gotten from MecanumDrivetrainClass
     */
    public Pedro_Paths(Follower follow){
        follower = follow;
    }
}
