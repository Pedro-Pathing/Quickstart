package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Drivetrain {

    private final Follower follower;

    public Drivetrain(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
    }

    /**
     * Start Pedro's teleop drive mode.
     */
    public void start() {
        follower.startTeleopDrive();
    }

    /**
     * Drive the robot using field-centric controls.
     *
     * @param forward Forward/backward joystick input
     * @param strafe  Left/right joystick input
     * @param turn    Rotation joystick input
     */
    public void drive(double forward, double strafe, double turn) {

        follower.setTeleOpDrive(
                forward,
                strafe,
                turn,
                true
        );
    }

    /**
     * Update Pedro Pathing.
     * This should be called once every loop.
     */
    public void update() {
        follower.update();
    }

    /**
     * Get the robot's current field position.
     */
    public Pose getPose() {
        return follower.getPose();
    }

    /**
     * Set the robot's starting position.
     */
    public void setStartingPose(Pose pose) {
        follower.setStartingPose(pose);
    }

    /**
     * Stop the drivetrain.
     */
    public void stop() {
        follower.breakFollowing();
    }
}