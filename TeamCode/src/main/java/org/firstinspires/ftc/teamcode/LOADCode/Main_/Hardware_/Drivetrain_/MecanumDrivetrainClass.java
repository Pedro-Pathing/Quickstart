package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Drivetrain_;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class MecanumDrivetrainClass {
    // Controls the speed of the robot
    public double speedMultiplier = 1.0; // make this slower for outreaches

    // Misc Constants
    public Follower follower = null;

    public static Pose robotPose = null;

    /**
     * Initializes the PedroPathing follower.
     * Needs to be run once after all hardware is initialized.
     * @param myOpMode Allows the follower access to the robot hardware.
     * @param initialPose The starting pose of the robot.
     */
    public void init (@NonNull OpMode myOpMode, Pose initialPose){
        // PedroPathing initialization
        follower = Constants.createFollower(myOpMode.hardwareMap);  // Initializes the PedroPathing path follower
        follower.setStartingPose(initialPose);                      // Sets the initial position of the robot on the field
        follower.update(); // Applies the initialization
    }

    /**
     * Initializes the PedroPathing follower.
     * Needs to be run once after all hardware is initialized.
     * @param myOpMode Allows the follower access to the robot hardware.
     * @param initialPose The starting pose of the robot.
     * @param follow The PedroPathing follower object
     */
    public void init (@NonNull OpMode myOpMode, Pose initialPose, Follower follow){
        // PedroPathing initialization
        follower = follow;  // Initializes the PedroPathing path follower
        follower.setPose(initialPose);                      // Sets the initial position of the robot on the field
        follower.update(); // Applies the initialization
    }

    public void startTeleOpDrive(){
        follower.startTeleopDrive();
        follower.update();
    }

    /**
     * Uses PedroPathing's follower class to implement a mecanum drive controller.
     * Must be called every loop to function properly.
     * @param forward The joystick value for driving forward/backward
     * @param strafe The joystick value for strafing
     * @param rotate The joystick value to turn left/right
     * @param robotCentric If true, enables robot centric. If false, enables field centric.
     */
    public void pedroMecanumDrive(double forward, double strafe, double rotate, boolean robotCentric){
        follower.setTeleOpDrive(
                -forward * speedMultiplier,
                -strafe * speedMultiplier,
                -rotate * speedMultiplier,
                robotCentric);
        follower.update();
    }

    public void runPath(PathChain path, boolean holdEndpoint){
        follower.followPath(path, holdEndpoint);
        follower.update();
    }

    public double distanceFromGoal(){
        Pose goalPose = new Pose(0,144,0);
        if (LoadHardwareClass.selectedAlliance == LoadHardwareClass.Alliance.RED) {goalPose = new Pose(144, 144, 0);}
        return follower.getPose().distanceFrom(goalPose);
    }

    public boolean pathComplete(){
        return !follower.isBusy();
    }
}