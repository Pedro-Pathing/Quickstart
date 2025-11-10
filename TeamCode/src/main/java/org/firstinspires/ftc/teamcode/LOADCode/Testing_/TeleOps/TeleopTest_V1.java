package org.firstinspires.ftc.teamcode.LOADCode.Testing_.TeleOps;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@TeleOp(name = "Example Robot-Centric TeleOp", group = "TestTeleOp")
public class TeleopTest_V1 extends OpMode {
    public static Follower follower;
    @IgnoreConfigurable
    static TelemetryManager telemetryM;
    /** Start Pose of our robot. This can be changed or saved from the autonomous period. */
    private final Pose startPose = new Pose(60,96, Math.toRadians(0));

    /** This method is called once when init is pressed and initializes the follower **/
    @Override
    public void init() {
        // Initializing the follower and setting its starting position.
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        // Calling this method is necessary at the start of your TeleOp OpMode.
        follower.startTeleopDrive();
        follower.update();
    }

    /** This is the main loop of the OpMode and runs continuously after pressing play **/
    @Override
    public void loop() {
        // Update robot movement based on gamepad inputs
        /* Update Pedro to move the robot based on:
         * Forward/Backward Movement: -gamepad1.left_stick_y
         * Left/Right Movement: -gamepad1.left_stick_x
         * Turn Left/Right Movement: -gamepad1.right_stick_x
         * Robot-Centric Mode: true
         */
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        // Loop robot movement and odometry values
        follower.update();

        // Telemetry Outputs of the Follower
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(1));

        // Update Telemetry to the Driver Hub
        telemetry.update();

        // Updating the status of the gamepad buttons
    }
}