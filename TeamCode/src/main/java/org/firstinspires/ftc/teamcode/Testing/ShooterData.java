package org.firstinspires.ftc.teamcode.Testing;
import com.acmerobotics.dashboard.config.Config;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Mechanics.Robot;
import org.firstinspires.ftc.teamcode.Mechanics.Shooter;
import org.firstinspires.ftc.teamcode.Mechanics.Spind;
import org.firstinspires.ftc.teamcode.Mechanics.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Config
@TeleOp(name = "shooterData", group = "testing")
public class ShooterData extends OpMode {
    public static double power = 0;
    private Follower follower;
    public static Pose startingPose = new Pose(9, 8.25, 0); //See ExampleAuto to understand how to use this
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private boolean cooked = true;
    private boolean julian = true;
    private boolean coooooking = true;
    private boolean coooooking2 = true;
    private double spindPos = 0;

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
        follower.startTeleopDrive();
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

        //Optional way to change slow mode strength
        if (gamepad1.xWasPressed()) {
            slowModeMultiplier += 0.25;
        }

        //Optional way to change slow mode strength
        if (gamepad2.right_bumper && cooked) {
            Shooter.setHood(Shooter.getHood() + .01);
            cooked = false;
        }
        else if (!gamepad2.right_bumper){
            cooked = true;
        }
        if (gamepad2.left_bumper && julian) {
            Shooter.setHood(Shooter.getHood() - .01);
            julian = false;
        }
        else if (!gamepad2.left_bumper){
            julian = true;
        }
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
            Shooter.setPower(0);
        if (gamepad2.b)
            Robot.flywheel.setPower(power);
        if (gamepad2.left_trigger > .6)
            Robot.flywheel.setPower(-.3);

        telemetry.addData("distance", Math.sqrt(Math.pow(144 - follower.getPose().getX(),2) + Math.pow(144 - follower.getPose().getY(), 2)));
        telemetry.addData("velocity", Shooter.getVel());
        telemetry.addData("hood", Shooter.getHood());
        Robot.intake.setPower(gamepad2.left_stick_y * Math.abs(gamepad2.left_stick_y));

        Spind.spinTheDexer(spindPos);

        double t = Turret.faceGoal(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading(), true, 0);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("theta", follower.getHeading());
        telemetry.addData("turret theta", t);
        telemetry.update();
    }
}