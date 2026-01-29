package org.firstinspires.ftc.teamcode.Teleop;
import static org.firstinspires.ftc.teamcode.Mechanics.Robot.autoEnd;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Mechanics.Robot;
import org.firstinspires.ftc.teamcode.Mechanics.Shooter;
import org.firstinspires.ftc.teamcode.Mechanics.Spind;
import org.firstinspires.ftc.teamcode.Mechanics.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "Blue TeleOp 🥶", group = "Comp Ready")
public class BlueT extends OpMode {
    private int x = 1;
    private boolean coooooking = true;
    private boolean coooooking2 = true;

    private double spindPos = 0;
    private Follower follower;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private boolean manual = true;

    private double off = 0;
    private boolean offsetswitch = true;
    private boolean offsetswitch2 = true;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(autoEnd);
        follower.update();

//        Robot.init(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();
        //This is the normal version to use in the TeleOp
        if (!slowMode) follower.setTeleOpDrive(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false // Robot Centric
        );

            //This is how it looks with slowMode on
        else follower.setTeleOpDrive(
                -gamepad1.left_stick_y * slowModeMultiplier,
                -gamepad1.left_stick_x * slowModeMultiplier,
                gamepad1.right_stick_x * slowModeMultiplier,
                false // Robot Centric
        );

        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        //Optional way to change slow mode strength
        if (gamepad1.xWasPressed()) {
            slowModeMultiplier += 0.25;
        }

        //Optional way to change slow mode strength
        if (gamepad1.yWasPressed()) {
            slowModeMultiplier -= 0.25;
        }



        //driver 2
        if (gamepad2.startWasPressed()) {
            manual = !manual;
        }
        if (gamepad2.right_bumper)
            Shooter.setHood(0.2);
//        if (gamepad2.left_bumper)
//            Shooter.setHood(.5);
        if (gamepad2.right_trigger > .6) {
            Robot.transfer.setPosition(.9);
            try { Thread.sleep(100); } catch (Exception ignored) {}
            Robot.transfer.setPosition(.4);
        }
        if (gamepad2.y && coooooking) {
            spindPos += .5;
//            int target = (int)Math.round((Constants.CPR / 6.0) * (x));
//            Robot.spindexer.setTargetPosition(target);
//            Robot.spindexer.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            Robot.spindexer.setPower(1);
//            x += 1;
            coooooking = false;
        }
        else if (!gamepad2.y)
            coooooking = true;
        if (gamepad2.a && coooooking2){
//            int target = (int)Math.round((Constants.CPR / 6.0) * (x+1));
//            Robot.spindexer.setTargetPosition(target);
//            Robot.spindexer.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            Robot.spindexer.setPower(1);
//            x += 2;
            spindPos += 1;
            coooooking2 = false;
        }
        else if (!gamepad2.a)
            coooooking2 = true;
        if (gamepad2.x)
            Robot.flywheel.setPower(1);
        if (gamepad2.b)
            Robot.flywheel.setPower(0);
        if (gamepad2.left_trigger > .6)
            Robot.flywheel.setPower(-.5);

        if (gamepad2.right_bumper && offsetswitch) {
            off += Math.toRadians(1);
            offsetswitch = false;
        }
        else if (!gamepad2.right_bumper) {
            offsetswitch = true;
        }
        if (gamepad2.left_bumper && offsetswitch2) {
            off -= Math.toRadians(1);
            offsetswitch2 = false;
        }
        else if (!gamepad2.left_bumper) {
            offsetswitch2 = true;
        }


        Robot.intake.setPower(gamepad2.left_stick_y * Math.abs(gamepad2.left_stick_y));

        Shooter.autoShotHood(follower.getPose().getX(), 144 - follower.getPose().getY(), follower.getHeading(), false);

        Spind.spinTheDexer(spindPos,true);

        Turret.faceGoal(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading(), false, off);



        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetry
                .addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("theta", follower.getHeading());
        telemetry.update();
    }
}