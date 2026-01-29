package org.firstinspires.ftc.teamcode.Testing;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanics.Robot;
import org.firstinspires.ftc.teamcode.Mechanics.Shooter;
import org.firstinspires.ftc.teamcode.Mechanics.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp
public class Test extends OpMode {
    private Follower follower;
    public static Pose startingPose = new Pose(72, 72, 0); //See ExampleAuto to understand how to use this
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        Robot.init(hardwareMap);
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();
        //This is the normal version to use in the TeleOp
        if (!slowMode) follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false // Robot Centric
        );

            //This is how it looks with slowMode on
        else follower.setTeleOpDrive(
                -gamepad1.left_stick_y * slowModeMultiplier,
                -gamepad1.left_stick_x * slowModeMultiplier,
                -gamepad1.right_stick_x * slowModeMultiplier,
                false // Robot Centric
        );

        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());


        if (gamepad2.x) {
            Shooter.setPower(1);
        }
        if (gamepad2.b) {
            Shooter.setPower(0);
        }
        if (gamepad2.right_trigger > .6) {
            Robot.transfer.setPosition(.9);
            try { Thread.sleep(100); } catch (Exception ignored) {}
            Robot.transfer.setPosition(.4);
        }


        double t = Turret.faceGoal(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading(), true, 0);
        Shooter.autoShotHood(144 - follower.getPose().getX(), 144 - follower.getPose().getY(), follower.getHeading(), true);

        telemetry.addData("hood", Shooter.autoShotHood(144 - follower.getPose().getX(), 144 - follower.getPose().getY(), follower.getHeading(), true));
        telemetry.addData("velocity", Shooter.getVel());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("theta", follower.getHeading());
        telemetry.addData("turret theta", t);
        telemetry.update();
    }
}