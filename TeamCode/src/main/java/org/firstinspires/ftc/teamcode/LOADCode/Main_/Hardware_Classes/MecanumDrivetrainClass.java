package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_Classes;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class MecanumDrivetrainClass {
    // Controls the speed of the robot
    public double speedMultiplier = 1.0; // make this slower for outreaches

    // Misc Constants
    private Follower follower = null;
    private Pose initialPose = new Pose(0,0, 0);

    /**
     * Sets the initial position & heading of the robot.
     * Must be run BEFORE the robot is intialized.
     * Otherwise, it will default to (0,0,0).
     * @param pose The position and heading that the robot starts at on the field.
     */
    public void setInitialPose (Pose pose){
        initialPose = pose;
    }

    /**
     * Initializes the PedroPathing follower.
     * Needs to be run once after all hardware is initialized.
     * @param myOpMode Allows the follower access to the robot hardware.
     */
    public void init (@NonNull OpMode myOpMode){
        // PedroPathing initialization
        follower = Constants.createFollower(myOpMode.hardwareMap);  // Initializes the PedroPathing path follower
        follower.setStartingPose(initialPose);                      // Sets the initial position of the robot on the field
        follower.update();                                          // Applies the initialization
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
                forward * speedMultiplier,
                strafe * speedMultiplier,
                rotate * speedMultiplier,
                robotCentric);
        follower.update();
    }
}