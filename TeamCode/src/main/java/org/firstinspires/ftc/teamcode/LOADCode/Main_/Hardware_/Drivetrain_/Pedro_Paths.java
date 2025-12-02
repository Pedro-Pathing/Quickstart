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
    // Dummy Stand-In Poses
    public Pose dummyStartPose = new Pose(0,0,0);
    public Pose dummyMoveRPPose = new Pose(0,28,0);
    // Start Poses
    public Pose startPose1 = new Pose(112, 136.6, Math.toRadians(270));
    public Pose startPose2 = new Pose(88, 7.4, Math.toRadians(90));
    // Preload Poses
    public Pose preloadPose1 = new Pose(130.000, 83.500, Math.toRadians(90));
    public Pose preloadPose2 = new Pose(132.000, 59.500, Math.toRadians(90));
    public Pose preloadPose3 = new Pose(132.000, 35.500, Math.toRadians(90));
    // Shooting Poses
    public Pose shootingPose1 = new Pose(115, 120, Math.toRadians(-35));
    public Pose shootingPose2 = new Pose(85, 85, Math.toRadians(-15));
    public Pose shootingPose3 = new Pose(85, 15, Math.toRadians(60));
    // Leave Poses
    public Pose leavePose1 = new Pose(90,120);
    public Pose leavePose2 = new Pose(95,55);
    public Pose leavePose3 = new Pose(115,20);

    // Define all path variables
    // Dummy Paths
    public PathChain basicMoveRPPath;
    // Start Poses to Preloads
    public PathChain startPose1_to_preload1, startPose1_to_preload2, startPose1_to_preload3;
    public PathChain startPose2_to_preload1, startPose2_to_preload2, startPose2_to_preload3;
    // Preloads to Shooting Positions
    public PathChain preload1_to_shooting1, preload1_to_shooting2, preload1_to_shooting3;
    public PathChain preload2_to_shooting1, preload2_to_shooting2, preload2_to_shooting3;
    public PathChain preload3_to_shooting1, preload3_to_shooting2, preload3_to_shooting3;
    // Shooting Positions to Preloads
    public PathChain shooting1_to_preload1, shooting1_to_preload2, shooting1_to_preload3;
    public PathChain shooting2_to_preload1, shooting2_to_preload2, shooting2_to_preload3;
    public PathChain shooting3_to_preload1, shooting3_to_preload2, shooting3_to_preload3;
    // Shooting Positions to Leave Positions
    public PathChain shooting1_to_leave1, shooting1_to_leave2;
    public PathChain shooting2_to_leave1, shooting2_to_leave2;
    public PathChain shooting3_to_leave2, shooting3_to_leave3;
    // Start Positions to Leave Positions
    public PathChain start1_to_leave1, start1_to_leave2;
    public PathChain start2_to_leave2, start2_to_leave3;

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
     * <h1>DON'T EVER USE THIS PATH!!!</h1>
     * This path is ONLY for the December 6th scrimmage, for movement RP bonus only!
     */
    public void buildMoveRPPath(){
        basicMoveRPPath = follower.pathBuilder()
                .addPath(new BezierCurve(dummyStartPose,dummyMoveRPPose))
                .setLinearHeadingInterpolation(dummyStartPose.getHeading(), dummyMoveRPPose.getHeading())
                .build();
    }
    public void buildStart1ToPreloads(LoadHardwareClass.Alliance alliance) {
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
                        autoMirror(preloadPose1, alliance)
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
                        autoMirror(preloadPose2, alliance)
                ))
                .setTangentHeadingInterpolation()
                .build();
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
                        autoMirror(preloadPose3, alliance)
                ))
                .setTangentHeadingInterpolation()
                .build();
    }
    public void buildStart2ToPreloads(LoadHardwareClass.Alliance alliance){
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
                        autoMirror(preloadPose1, alliance)
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
                        autoMirror(preloadPose2, alliance)
                ))
                .setTangentHeadingInterpolation()
                .build();
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
                        autoMirror(preloadPose3, alliance)
                ))
                .setTangentHeadingInterpolation()
                .build();
    }
    public void buildPreload1ToShootings(LoadHardwareClass.Alliance alliance){
        preload1_to_shooting1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(preloadPose1, alliance),
                        autoMirror(shootingPose1, alliance)
                ))
                .setLinearHeadingInterpolation(preloadPose1.getHeading(), shootingPose1.getHeading())
                .build();
        preload1_to_shooting2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(preloadPose1, alliance),
                        autoMirror(shootingPose2, alliance)
                ))
                .setLinearHeadingInterpolation(preloadPose1.getHeading(), shootingPose2.getHeading())
                .build();
        preload1_to_shooting3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(preloadPose1, alliance),
                        autoMirror(new Pose(80.000, 83.500), alliance),
                        autoMirror(shootingPose3, alliance)
                ))
                .setLinearHeadingInterpolation(preloadPose1.getHeading(), shootingPose3.getHeading())
                .build();
    }
    public void buildPreload2ToShootings(LoadHardwareClass.Alliance alliance){
        preload2_to_shooting1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(preloadPose2, alliance),
                        autoMirror(new Pose(65,59.5), alliance),
                        autoMirror(shootingPose1, alliance)
                ))
                .setLinearHeadingInterpolation(preloadPose2.getHeading(), shootingPose1.getHeading())
                .build();
        preload2_to_shooting2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(preloadPose2, alliance),
                        autoMirror(new Pose(65,59.5), alliance),
                        autoMirror(shootingPose2, alliance)
                ))
                .setLinearHeadingInterpolation(preloadPose2.getHeading(), shootingPose2.getHeading())
                .build();
        preload2_to_shooting3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(preloadPose2, alliance),
                        autoMirror(new Pose(90.000, 59.500), alliance),
                        autoMirror(shootingPose3, alliance)
                ))
                .setLinearHeadingInterpolation(preloadPose2.getHeading(), shootingPose3.getHeading())
                .build();
    }
    public void buildPreload3ToShootings(LoadHardwareClass.Alliance alliance){
        preload3_to_shooting1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(preloadPose2, alliance),
                        autoMirror(new Pose(75,30), alliance),
                        autoMirror(new Pose(80,100), alliance),
                        autoMirror(shootingPose1, alliance)
                ))
                .setLinearHeadingInterpolation(preloadPose2.getHeading(), shootingPose1.getHeading())
                .build();
        preload3_to_shooting2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(preloadPose2, alliance),
                        autoMirror(new Pose(85,32), alliance),
                        autoMirror(shootingPose2, alliance)
                ))
                .setLinearHeadingInterpolation(preloadPose2.getHeading(), shootingPose2.getHeading())
                .build();
        preload3_to_shooting3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(preloadPose2, alliance),
                        autoMirror(shootingPose3, alliance)
                ))
                .setLinearHeadingInterpolation(preloadPose2.getHeading(), shootingPose3.getHeading())
                .build();
    }
    public void buildShooting1ToPreloads(LoadHardwareClass.Alliance alliance){
        shooting1_to_preload1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(shootingPose1, alliance),
                        autoMirror(new Pose(60, 80), alliance),
                        autoMirror(preloadPose1, alliance)
                ))
                .setLinearHeadingInterpolation(shootingPose1.getHeading(), preloadPose1.getHeading())
                .build();
        shooting1_to_preload2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(shootingPose1, alliance),
                        autoMirror(new Pose(60, 55), alliance),
                        autoMirror(preloadPose2, alliance)
                ))
                .setLinearHeadingInterpolation(shootingPose1.getHeading(), preloadPose2.getHeading())
                .build();
        shooting1_to_preload3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(shootingPose1, alliance),
                        autoMirror(new Pose(60, 27), alliance),
                        autoMirror(preloadPose3, alliance)
                ))
                .setLinearHeadingInterpolation(shootingPose1.getHeading(), preloadPose3.getHeading())
                .build();
    }
    public void buildShooting2ToPreloads(LoadHardwareClass.Alliance alliance){
        shooting2_to_preload1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(shootingPose2, alliance),
                        autoMirror(preloadPose1, alliance)
                ))
                .setLinearHeadingInterpolation(shootingPose2.getHeading(), preloadPose1.getHeading())
                .build();
        shooting2_to_preload2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(shootingPose2, alliance),
                        autoMirror(new Pose(75, 56), alliance),
                        autoMirror(preloadPose2, alliance)
                ))
                .setLinearHeadingInterpolation(shootingPose2.getHeading(), preloadPose2.getHeading())
                .build();
        shooting2_to_preload3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(shootingPose2, alliance),
                        autoMirror(new Pose(68, 30), alliance),
                        autoMirror(preloadPose3, alliance)
                ))
                .setLinearHeadingInterpolation(shootingPose2.getHeading(), preloadPose3.getHeading())
                .build();
    }
    public void buildShooting3ToPreloads(LoadHardwareClass.Alliance alliance){
        shooting3_to_preload1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(shootingPose3, alliance),
                        autoMirror(new Pose(73, 88), alliance),
                        autoMirror(preloadPose1, alliance)
                ))
                .setLinearHeadingInterpolation(shootingPose3.getHeading(), preloadPose1.getHeading())
                .build();
        shooting3_to_preload2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(shootingPose3, alliance),
                        autoMirror(new Pose(78, 62), alliance),
                        autoMirror(preloadPose2, alliance)
                ))
                .setLinearHeadingInterpolation(shootingPose3.getHeading(), preloadPose2.getHeading())
                .build();
        shooting3_to_preload3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(shootingPose3, alliance),
                        autoMirror(new Pose(82.5, 35), alliance),
                        autoMirror(preloadPose3, alliance)
                ))
                .setLinearHeadingInterpolation(shootingPose3.getHeading(), preloadPose3.getHeading())
                .build();
    }
    public void buildShooting1ToLeaves(LoadHardwareClass.Alliance alliance){
        shooting1_to_leave1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(shootingPose1, alliance),
                        autoMirror(leavePose1, alliance)
                ))
                .setConstantHeadingInterpolation(shootingPose1.getHeading())
                .build();
        shooting1_to_leave2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(shootingPose1, alliance),
                        autoMirror(leavePose2, alliance)
                ))
                .setConstantHeadingInterpolation(shootingPose1.getHeading())
                .build();
    }
    public void buildShooting2ToLeaves(LoadHardwareClass.Alliance alliance){
        shooting2_to_leave1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(shootingPose2, alliance),
                        autoMirror(leavePose1, alliance)
                ))
                .setConstantHeadingInterpolation(shootingPose2.getHeading())
                .build();
        shooting2_to_leave2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(shootingPose2, alliance),
                        autoMirror(leavePose2, alliance)
                ))
                .setConstantHeadingInterpolation(shootingPose2.getHeading())
                .build();
    }
    public void buildShooting3ToLeaves(LoadHardwareClass.Alliance alliance){
        shooting3_to_leave2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(shootingPose3, alliance),
                        autoMirror(leavePose2, alliance)
                ))
                .setConstantHeadingInterpolation(shootingPose3.getHeading())
                .build();
        shooting3_to_leave3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(shootingPose3, alliance),
                        autoMirror(leavePose3, alliance)
                ))
                .setConstantHeadingInterpolation(shootingPose3.getHeading())
                .build();
    }
    public void buildStart1ToLeaves(LoadHardwareClass.Alliance alliance){
        start1_to_leave1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(startPose1, alliance),
                        autoMirror(leavePose1, alliance)
                ))
                .setConstantHeadingInterpolation(startPose1.getHeading())
                .build();
        start1_to_leave2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(startPose1, alliance),
                        autoMirror(leavePose3, alliance)
                ))
                .setConstantHeadingInterpolation(startPose1.getHeading())
                .build();
    }
    public void buildStart2ToLeaves(LoadHardwareClass.Alliance alliance){
        start2_to_leave2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(startPose2, alliance),
                        autoMirror(leavePose1, alliance)
                ))
                .setConstantHeadingInterpolation(startPose2.getHeading())
                .build();
        start2_to_leave3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(startPose2, alliance),
                        autoMirror(leavePose3, alliance)
                ))
                .setConstantHeadingInterpolation(startPose2.getHeading())
                .build();
    }

    /**
     * Builds all the paths, mirroring them to the other side of the field if necessary
     */
    public void buildPaths(LoadHardwareClass.Alliance alliance){
        /// All paths are for the RED side of the field. they will be mirrored if necessary.
        buildMoveRPPath();
        // Paths going from each start position to each of the preloads.
        buildStart1ToPreloads(alliance);
        buildStart2ToPreloads(alliance);
        // Paths going from each preload to each shooting position
        buildPreload1ToShootings(alliance);
        buildPreload2ToShootings(alliance);
        buildPreload3ToShootings(alliance);
        // Paths going from each shooting position to each preload
        buildShooting1ToPreloads(alliance);
        buildShooting2ToPreloads(alliance);
        buildShooting3ToPreloads(alliance);
        // Paths going from each shooting position to the leave positions.
        buildShooting1ToLeaves(alliance);
        buildShooting2ToLeaves(alliance);
        buildShooting3ToLeaves(alliance);
        // Paths going from the start positions to the leave positions
        buildStart1ToLeaves(alliance);
        buildStart2ToLeaves(alliance);
    }

    /**
     * Must be called after MecanumDrivetrainClass is initialized.
     * @param follow PedroPathing's follower object, gotten from MecanumDrivetrainClass
     */
    public Pedro_Paths(Follower follow){
        follower = follow;
    }
}
