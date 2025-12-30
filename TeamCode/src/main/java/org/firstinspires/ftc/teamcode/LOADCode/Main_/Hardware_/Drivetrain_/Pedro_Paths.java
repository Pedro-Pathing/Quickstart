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
    public Pose nearStart = new Pose(112, 136.6, Math.toRadians(270));
    public Pose farStart = new Pose(88, 7.4, Math.toRadians(90));
    // Preload Poses
    public Pose nearPreload = new Pose(128.000, 83.500, Math.toRadians(0));
    public Pose midPreload = new Pose(132.000, 59.500, Math.toRadians(0));
    public Pose farPreload = new Pose(132.000, 35.500, Math.toRadians(0));
    // Shooting Poses
    public Pose nearShoot = new Pose(115, 120, Math.toRadians(-35));
    public Pose midShoot = new Pose(85, 85, Math.toRadians(-15));
    public Pose farShoot = new Pose(85, 15, Math.toRadians(60));
    // Leave Poses
    public Pose nearLeave = new Pose(90,120, Math.toRadians(90));
    public Pose midLeave = new Pose(95,55, Math.toRadians(90));
    public Pose farLeave = new Pose(115,20, Math.toRadians(90));

    /**
     * <h4>Define all path variables</h4>
     */
    // Start Poses to Preloads
    public PathChain nearStart_to_nearPreload, nearStart_to_midPreload, nearStart_to_farPreload;
    public PathChain farStart_to_nearPreload, farStart_to_midPreload, farStart_to_farPreload;
    // Preloads to Shooting Positions
    public PathChain nearPreload_to_nearShoot, nearPreload_to_midShoot, nearPreload_to_farShoot;
    public PathChain midPreload_to_nearShoot, midPreload_to_midShoot, midPreload_to_farShoot;
    public PathChain farPreload_to_nearShoot, farPreload_to_midShoot, farPreload_to_farShoot;
    // Shooting Positions to Preloads
    public PathChain nearShoot_to_nearPreload, nearShoot_to_midPreload, nearShoot_to_farPreload;
    public PathChain midShoot_to_nearPreload, midShoot_to_midPreload, midShoot_to_farPreload;
    public PathChain farShoot_to_nearPreload, farShoot_to_midPreload, farShoot_to_farPreload;
    // Shooting Positions to Leave Positions
    public PathChain nearShoot_to_nearLeave, nearShoot_to_midLeave;
    public PathChain midShoot_to_nearLeave, midShoot_to_midLeave;
    public PathChain farShoot_to_midLeave, farShoot_to_farLeave;
    // Start Positions to Leave Positions
    public PathChain nearStart_to_nearLeave, nearStart_to_midLeave;
    public PathChain farStart_to_midLeave, farStart_to_farLeave;

    public Pose autoMirror(Pose pose, LoadHardwareClass.Alliance alliance){
        if (alliance == LoadHardwareClass.Alliance.BLUE){
            return new Pose(
                    144 - pose.getX(),
                    144 - pose.getY(),
                    Math.atan2(Math.sin(pose.getHeading()), -Math.cos(pose.getHeading()))
            );
        }
        return pose;
    }

    public void buildStart1ToPreloads(LoadHardwareClass.Alliance alliance) {
        nearStart_to_nearPreload = follower.pathBuilder() 
                .addPath(new BezierCurve(
                        autoMirror(nearStart, alliance),
                        autoMirror(new Pose(89.000, 136.600), alliance),
                        autoMirror(new Pose(89.000, 78.000), alliance),
                        autoMirror(new Pose(95.000, 83.500), alliance),
                        autoMirror(new Pose(110.000, 83.500), alliance)
                ))
                .setLinearHeadingInterpolation(nearStart.getHeading(), Math.toRadians(0))
                .addPath(new BezierLine(
                        autoMirror(new Pose(110.000, 83.500), alliance),
                        autoMirror(nearPreload, alliance)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        nearStart_to_midPreload = follower.pathBuilder() 
                .addPath(new BezierCurve(
                        autoMirror(nearStart, alliance),
                        autoMirror(new Pose(89.000, 136.600), alliance),
                        autoMirror(new Pose(89.000, 54.000), alliance),
                        autoMirror(new Pose(95.000, 59.500), alliance),
                        autoMirror(new Pose(110.000, 59.500), alliance)
                ))
                .setLinearHeadingInterpolation(nearStart.getHeading(), Math.toRadians(0))
                .addPath(new BezierLine(
                        autoMirror(new Pose(110.000, 59.500), alliance),
                        autoMirror(midPreload, alliance)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        nearStart_to_farPreload = follower.pathBuilder() 
                .addPath(new BezierCurve(
                        autoMirror(nearStart, alliance),
                        autoMirror(new Pose(89.000, 136.600), alliance),
                        autoMirror(new Pose(89.000, 30.000), alliance),
                        autoMirror(new Pose(95.000, 35.500), alliance),
                        autoMirror(new Pose(110.000, 35.500), alliance)
                ))
                .setLinearHeadingInterpolation(nearStart.getHeading(), Math.toRadians(0))
                .addPath(new BezierLine(
                        autoMirror(new Pose(110.000, 35.500), alliance),
                        autoMirror(farPreload, alliance)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }
    public void buildStart2ToPreloads(LoadHardwareClass.Alliance alliance){
        farStart_to_nearPreload = follower.pathBuilder() 
                .addPath(new BezierCurve(
                        autoMirror(farStart, alliance),
                        autoMirror(new Pose(89.000, 79.000), alliance),
                        autoMirror(new Pose(95.000, 83.500), alliance),
                        autoMirror(new Pose(110.000, 83.500), alliance)
                ))
                .setLinearHeadingInterpolation(farStart.getHeading(), Math.toRadians(0))
                .addPath(new BezierLine(
                        autoMirror(new Pose(110.000, 83.500), alliance),
                        autoMirror(nearPreload, alliance)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        farStart_to_midPreload = follower.pathBuilder() 
                .addPath(new BezierCurve(
                        autoMirror(farStart, alliance),
                        autoMirror(new Pose(89.000, 55.000), alliance),
                        autoMirror(new Pose(95.000, 59.500), alliance),
                        autoMirror(new Pose(110.000, 59.500), alliance)
                ))
                .setLinearHeadingInterpolation(nearStart.getHeading(), Math.toRadians(0))
                .addPath(new BezierLine(
                        autoMirror(new Pose(110.000, 59.500), alliance),
                        autoMirror(midPreload, alliance)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        farStart_to_farPreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(farStart, alliance),
                        autoMirror(new Pose(89.000, 25), alliance),
                        autoMirror(new Pose(95.000, 35.500), alliance),
                        autoMirror(new Pose(110.000, 35.500), alliance)
                ))
                .setLinearHeadingInterpolation(farStart.getHeading(), Math.toRadians(0))
                .addPath(new BezierLine(
                        autoMirror(new Pose(110.000, 35.500), alliance),
                        autoMirror(farPreload, alliance)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }
    public void buildPreload1ToShootings(LoadHardwareClass.Alliance alliance){
        nearPreload_to_nearShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(nearPreload, alliance),
                        autoMirror(nearShoot, alliance)
                ))
                .setLinearHeadingInterpolation(nearPreload.getHeading(), nearShoot.getHeading())
                .build();
        nearPreload_to_midShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(nearPreload, alliance),
                        autoMirror(midShoot, alliance)
                ))
                .setLinearHeadingInterpolation(nearPreload.getHeading(), midShoot.getHeading())
                .build();
        nearPreload_to_farShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(nearPreload, alliance),
                        autoMirror(new Pose(80.000, 83.500), alliance),
                        autoMirror(farShoot, alliance)
                ))
                .setLinearHeadingInterpolation(nearPreload.getHeading(), farShoot.getHeading())
                .build();
    }
    public void buildPreload2ToShootings(LoadHardwareClass.Alliance alliance){
        midPreload_to_nearShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(midPreload, alliance),
                        autoMirror(new Pose(65,59.5), alliance),
                        autoMirror(nearShoot, alliance)
                ))
                .setLinearHeadingInterpolation(midPreload.getHeading(), nearShoot.getHeading())
                .build();
        midPreload_to_midShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(midPreload, alliance),
                        autoMirror(new Pose(65,59.5), alliance),
                        autoMirror(midShoot, alliance)
                ))
                .setLinearHeadingInterpolation(midPreload.getHeading(), midShoot.getHeading())
                .build();
        midPreload_to_farShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(midPreload, alliance),
                        autoMirror(new Pose(90.000, 59.500), alliance),
                        autoMirror(farShoot, alliance)
                ))
                .setLinearHeadingInterpolation(midPreload.getHeading(), farShoot.getHeading())
                .build();
    }
    public void buildPreload3ToShootings(LoadHardwareClass.Alliance alliance){
        farPreload_to_nearShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(farPreload, alliance),
                        autoMirror(new Pose(75,30), alliance),
                        autoMirror(new Pose(80,100), alliance),
                        autoMirror(nearShoot, alliance)
                ))
                .setLinearHeadingInterpolation(midPreload.getHeading(), nearShoot.getHeading())
                .build();
        farPreload_to_midShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(farPreload, alliance),
                        autoMirror(new Pose(85,32), alliance),
                        autoMirror(midShoot, alliance)
                ))
                .setLinearHeadingInterpolation(midPreload.getHeading(), midShoot.getHeading())
                .build();
        farPreload_to_farShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(farPreload, alliance),
                        autoMirror(farShoot, alliance)
                ))
                .setLinearHeadingInterpolation(midPreload.getHeading(), farShoot.getHeading())
                .build();
    }
    public void buildShooting1ToPreloads(LoadHardwareClass.Alliance alliance){
        nearShoot_to_nearPreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(nearShoot, alliance),
                        autoMirror(new Pose(60, 80), alliance),
                        autoMirror(nearPreload, alliance)
                ))
                .setLinearHeadingInterpolation(nearShoot.getHeading(), nearPreload.getHeading())
                .build();
        nearShoot_to_midPreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(nearShoot, alliance),
                        autoMirror(new Pose(60, 55), alliance),
                        autoMirror(midPreload, alliance)
                ))
                .setLinearHeadingInterpolation(nearShoot.getHeading(), midPreload.getHeading())
                .build();
        nearShoot_to_farPreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(nearShoot, alliance),
                        autoMirror(new Pose(60, 27), alliance),
                        autoMirror(farPreload, alliance)
                ))
                .setLinearHeadingInterpolation(nearShoot.getHeading(), farPreload.getHeading())
                .build();
    }
    public void buildShooting2ToPreloads(LoadHardwareClass.Alliance alliance){
        midShoot_to_nearPreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(midShoot, alliance),
                        autoMirror(nearPreload, alliance)
                ))
                .setLinearHeadingInterpolation(midShoot.getHeading(), nearPreload.getHeading())
                .build();
        midShoot_to_midPreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(midShoot, alliance),
                        autoMirror(new Pose(75, 56), alliance),
                        autoMirror(midPreload, alliance)
                ))
                .setLinearHeadingInterpolation(midShoot.getHeading(), midPreload.getHeading())
                .build();
        midShoot_to_farPreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(midShoot, alliance),
                        autoMirror(new Pose(68, 30), alliance),
                        autoMirror(farPreload, alliance)
                ))
                .setLinearHeadingInterpolation(midShoot.getHeading(), farPreload.getHeading())
                .build();
    }
    public void buildShooting3ToPreloads(LoadHardwareClass.Alliance alliance){
        farShoot_to_nearPreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(farShoot, alliance),
                        autoMirror(new Pose(73, 88), alliance),
                        autoMirror(nearPreload, alliance)
                ))
                .setLinearHeadingInterpolation(farShoot.getHeading(), nearPreload.getHeading())
                .build();
        farShoot_to_midPreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(farShoot, alliance),
                        autoMirror(new Pose(78, 62), alliance),
                        autoMirror(midPreload, alliance)
                ))
                .setLinearHeadingInterpolation(farShoot.getHeading(), midPreload.getHeading())
                .build();
        farShoot_to_farPreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(farShoot, alliance),
                        autoMirror(new Pose(82.5, 35), alliance),
                        autoMirror(farPreload, alliance)
                ))
                .setLinearHeadingInterpolation(farShoot.getHeading(), farPreload.getHeading())
                .build();
    }
    public void buildShooting1ToLeaves(LoadHardwareClass.Alliance alliance){
        nearShoot_to_nearLeave = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(nearShoot, alliance),
                        autoMirror(nearLeave, alliance)
                ))
                .setConstantHeadingInterpolation(nearShoot.getHeading())
                .build();
        nearShoot_to_midLeave = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(nearShoot, alliance),
                        autoMirror(midLeave, alliance)
                ))
                .setConstantHeadingInterpolation(nearShoot.getHeading())
                .build();
    }
    public void buildShooting2ToLeaves(LoadHardwareClass.Alliance alliance){
        midShoot_to_nearLeave = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(midShoot, alliance),
                        autoMirror(nearLeave, alliance)
                ))
                .setConstantHeadingInterpolation(midShoot.getHeading())
                .build();
        midShoot_to_midLeave = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(midShoot, alliance),
                        autoMirror(midLeave, alliance)
                ))
                .setConstantHeadingInterpolation(midShoot.getHeading())
                .build();
    }
    public void buildShooting3ToLeaves(LoadHardwareClass.Alliance alliance){
        farShoot_to_midLeave = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(farShoot, alliance),
                        autoMirror(midLeave, alliance)
                ))
                .setConstantHeadingInterpolation(farShoot.getHeading())
                .build();
        farShoot_to_farLeave = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(farShoot, alliance),
                        autoMirror(farLeave, alliance)
                ))
                .setConstantHeadingInterpolation(farShoot.getHeading())
                .build();
    }
    public void buildStart1ToLeaves(LoadHardwareClass.Alliance alliance){
        nearStart_to_nearLeave = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(nearStart, alliance),
                        autoMirror(nearLeave, alliance)
                ))
                .setConstantHeadingInterpolation(nearStart.getHeading())
                .build();
        nearStart_to_midLeave = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(nearStart, alliance),
                        autoMirror(farLeave, alliance)
                ))
                .setConstantHeadingInterpolation(nearStart.getHeading())
                .build();
    }
    public void buildStart2ToLeaves(LoadHardwareClass.Alliance alliance){
        farStart_to_midLeave = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(farStart, alliance),
                        autoMirror(nearLeave, alliance)
                ))
                .setConstantHeadingInterpolation(farStart.getHeading())
                .build();
        farStart_to_farLeave = follower.pathBuilder()
                .addPath(new BezierCurve(
                        autoMirror(farStart, alliance),
                        autoMirror(farLeave, alliance)
                ))
                .setConstantHeadingInterpolation(farStart.getHeading())
                .build();
    }

    /**
     * Builds all the paths, mirroring them to the other side of the field if necessary
     */
    public void buildPaths(LoadHardwareClass.Alliance alliance, Follower follow){
        follower = follow;

        /// All paths are for the RED side of the field. they will be mirrored if necessary.
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
}
