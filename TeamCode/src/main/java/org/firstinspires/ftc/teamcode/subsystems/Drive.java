package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utils.Logger;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class Drive implements Subsystem {
    public static final Drive INSTANCE = new Drive();
    private static Follower follower;
    public static TelemetryManager telemetryM;
    private static boolean slowMode = false;
    private static final double slowModeMultiplier = 0.2;
    private static final boolean robotCentric = false;
    private static Pose startingPose = new Pose(24, 24, Math.toRadians(0));
    private static Pose shootTarget = new Pose(6, 144 - 6, 0);
    private static double headingGoal; // Radians
    private static PIDFController controller;
    private static boolean headingLock = false;



    @Override
    public void initialize() {
        follower = Constants.createFollower(ActiveOpMode.hardwareMap());
        follower.setStartingPose(new Pose(68, 76, Math.toRadians(315)));
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        setShootTarget();
        controller = new PIDFController(new PIDFCoefficients(1, 0, 0, 0));
    }

    @Override
    public void periodic() {
        drive.schedule();
    }

    public static void setHeadingLock(boolean lock){
        headingLock = lock;
    }
    private static void setHeadingGoal(Pose targetPose, Pose robotPose) {
        headingGoal = Math.atan2(targetPose.getY() - robotPose.getY(), targetPose.getX() - robotPose.getX());

    }
    private static double getHeadingError() {
        return MathFunctions.getTurnDirection(follower.getPose().getHeading(), headingGoal) * MathFunctions.getSmallestAngleDifference(follower.getPose().getHeading(), headingGoal);
    }

    private static void setShootTarget() {
        if (Robot.getCurrentAlliance() == Robot.Alliance.BLUE && shootTarget.getX() != 6)
            shootTarget = new Pose(6, 144 - 6, 0);
        else if (Robot.getCurrentAlliance() == Robot.Alliance.RED && shootTarget.getX() != (144 - 6))
            shootTarget = shootTarget.mirror();
    }

    private static void setSlowMode(boolean newMode) {
        slowMode = newMode;
    }

    public static Command setSlowModeCommand(boolean newMode) {
        return new InstantCommand(() -> setSlowMode(newMode));
    }

    public static void resetDrive() {
        if (Robot.getCurrentAlliance().equals(Robot.Alliance.BLUE)) {
            follower.setPose(new Pose(8, 6.25, Math.toRadians(0)).mirror());
        } else {
            follower.setPose(new Pose(8, 6.25, Math.toRadians(0)));
        }
    }

    public static Command resetDriveCommand() {
        return new InstantCommand(Drive::resetDrive);
    }
    public static Command drive = new LambdaCommand()
            .setStart(() -> follower.startTeleopDrive())
            .setUpdate(() -> {
                follower.update();
                telemetryM.update();

                // Calculate the correct values based on Gamepad 1
                double forward = slowMode ? ActiveOpMode.gamepad1().left_stick_y * slowModeMultiplier: ActiveOpMode.gamepad1().left_stick_y;
                double strafe = slowMode ? ActiveOpMode.gamepad1().left_stick_x * slowModeMultiplier: ActiveOpMode.gamepad1().left_stick_x;
                double turn = slowMode ? -ActiveOpMode.gamepad1().right_stick_x * slowModeMultiplier: -ActiveOpMode.gamepad1().right_stick_x;

                if (headingLock) {
                    setHeadingGoal(shootTarget, follower.getPose());
                    controller.updateError(getHeadingError());

                    follower.setTeleOpDrive(forward, strafe, controller.run(), robotCentric);
                } else {
                    follower.setTeleOpDrive(forward, strafe, turn, robotCentric);
                }

                Logger.add("Drive", Logger.Level.DEBUG, "forward: " + forward + " strafe: " + strafe + " turn: " + turn);
                Logger.add("Drive", Logger.Level.DEBUG, "slowmode? " + slowMode + "multiplier? " + slowModeMultiplier);
            })
            .setStop(interrupted -> {})
            .setIsDone(() -> false)
            .requires(Drive.INSTANCE)
            .setInterruptible(false)
            .named("Drive");
}
