package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Drivetrain_;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Pedro_Paths {
    // The variable to store PedroPathing's follower object for path building
    private Follower follower;

    /**
     * Must be called after MecanumDrivetrainClass is initialized.
     * @param follow PedroPathing's follower object, gotten from MecanumDrivetrainClass
     */
    public void init(Follower follow){
        follower = follow;
    }

    // Define primary poses to be used in paths
    public final Pose startPose1 = new Pose(87, 8.8, Math.toRadians(90));
    public final Pose scorePose1 = new Pose(86, 22, Math.toRadians(80));

    // Define paths to be used by PedroPathing
    public final PathChain gotoScorePose1 = follower.pathBuilder()
            .addPath(new BezierCurve(
                    follower.getPose(),
                    scorePose1
            ))
            .setLinearHeadingInterpolation(follower.getPose().getHeading(), scorePose1.getHeading())
            .build();
}
