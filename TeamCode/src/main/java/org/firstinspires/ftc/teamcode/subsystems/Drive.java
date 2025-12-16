package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Drive implements Subsystem {
    public static final Drive INSTANCE = new Drive();
    private static Follower follower;
    public static Pose startingPose;
    private static TelemetryManager telemetryM;
    private static boolean slowMode = false;
    private static double slowModeMultiplier = 0.5;
    private static boolean robotCentric = true;

    @Override
    public void initialize() {
        // runs only once, when OpMode initializes
        follower = Constants.createFollower(ActiveOpMode.hardwareMap());

        // TODO: update this to include the correct position if robotCentric is what we want
        if (robotCentric){
            startingPose = new Pose(12,12, Math.toRadians(90));
            // Pose = Cartesian coordinate system (standard x,y,θ) with bottom left facing left as (0,0,0)
        }
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower.update();
    }

    @Override
    public void periodic() {
        // This runs every loop, attempts to schedule the drive command
        drive.schedule();
    }

    public static Command drive = new LambdaCommand()
            .setStart(() -> follower.startTeleopDrive())
            .setUpdate(() -> {
                follower.update();
                telemetryM.update();

                // Calculate the correct values based on Gamepad 1
                double forward = slowMode ? -ActiveOpMode.gamepad1().left_stick_y : -ActiveOpMode.gamepad1().left_stick_y * slowModeMultiplier;
                double strafe = slowMode ? -ActiveOpMode.gamepad1().left_stick_x : -ActiveOpMode.gamepad1().left_stick_x * slowModeMultiplier;
                double turn = slowMode ? -ActiveOpMode.gamepad1().right_stick_x : -ActiveOpMode.gamepad1().right_stick_x * slowModeMultiplier;

                // Uses PedroPathing's Follower to drive the robot <- pedroPathing/Constants for more info
                follower.setTeleOpDrive(forward, strafe, turn, robotCentric);

                if (ActiveOpMode.gamepad1().rightBumperWasPressed()) {
                    slowMode = !slowMode;
                }

                //log verbosely to telemetry
                //ActiveOpMode.telemetry().addLine("forward: " + forward + " strafe: " + strafe + " turn: " + turn);
                //ActiveOpMode.telemetry().addData("slowMode?", slowMode);
                ActiveOpMode.telemetry().update();
            })
            .setStop(interrupted -> {})
            .setIsDone(() -> false)
            .requires(Drive.INSTANCE)
            .setInterruptible(false)
            .named("Drive");
}
