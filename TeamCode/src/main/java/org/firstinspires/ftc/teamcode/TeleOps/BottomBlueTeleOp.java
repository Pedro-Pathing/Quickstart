package org.firstinspires.ftc.teamcode.TeleOps;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.*;
import com.pedropathing.math.*;
import com.pedropathing.paths.*;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class BottomBlueTeleOp extends OpMode {

    int intakeFlag = 0;
    int gateFlag = 1;

    private DcMotor intake;
    private Servo /*liftleft, liftright,*/ gate, flick, hold,  light;  // servos
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private Supplier<PathChain> pathChain;

    private TelemetryManager telemetryM;
    private ShooterSubsystem shooter;
    private double lastTime = 0.0;
    private Timer flickTimer;


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose(72,72,90) : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        intake = hardwareMap.get(DcMotor.class, "intake");
        // liftleft = hardwareMap.get(Servo.class, "liftleft");
        //   liftright = hardwareMap.get(Servo.class, "liftright");
        gate = hardwareMap.get(Servo.class, "gate");
        flick = hardwareMap.get(Servo.class,"flick");
        hold = hardwareMap.get(Servo.class, "hold");
        light = hardwareMap.get(Servo.class, "light");

        // Shooter subsystem
        shooter = new ShooterSubsystem(hardwareMap);

        flickTimer = new Timer();

        // Example path (optional)
        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierCurve(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
        telemetry.addLine("Initialized");

        gate.setPosition(0.3);
        flick.setPosition(1);
        hold.setPosition(0.3);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        lastTime = getRuntime();
        shooter.getLimelight().start();
        shooter.getLimelight().pipelineSwitch(1); // 1 is for blue tracking

    }

    @Override
    public void loop() {

        follower.update();
        telemetryM.update();

        // TeleOp driving
        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors
            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );
                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }


        // if (detectedColorBottom == ColorSensorBottom.DetectedColor.GREEN)

        // launcher update
        double currentTime = getRuntime();
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        shooter.update(follower.getPose(), follower.getVelocity(), dt, telemetry);

        double tps1 = shooter.getFlywheel1().getVelocity();
        double tps2 = shooter.getFlywheel2().getVelocity();
        double rpm1 = (tps1 / 28) * 60;
        double rpm2 = (tps2 / 28) * 60;

        double avgRPM = (rpm1 + rpm2) / 2;


        telemetry.addData("Flywheel RPM", avgRPM);
        telemetry.addData("Hood Position", shooter.getHood().getPosition());
        telemetry.addData("Limelight tA", shooter.getLimelight().getLatestResult().getTa());
        telemetry.update();


        if (gamepad1.dpadLeftWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }
        if (automatedDrive && (gamepad1.dpadRightWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        if (gamepad1.xWasPressed()){
            flickTimer.resetTimer();
            flick.setPosition(0.6);  //0 is up
            sleep(300);
            flick.setPosition(1);
        }

        if (gamepad1.aWasPressed()) {   // intake in
            if (intakeFlag == 0) {
                intake.setPower(1);
                hold.setPosition(0.5);
                intakeFlag = 1;
            }
            else if (intakeFlag == 1){
                intake.setPower(0);
                hold.setPosition(0.3);
                intakeFlag = 0;
            }
            else if (intakeFlag == -1){
                intake.setPower(0);
                hold.setPosition(0.3);
                intakeFlag = 0;
            }
        }

        if (gamepad1.bWasPressed()) {   // intake out
            if (intakeFlag == 0) {
                intake.setPower(-1);
                hold.setPosition(0.5);
                intakeFlag = -1;
            }
            else if (intakeFlag == -1){
                intake.setPower(0);
                hold.setPosition(0.3);
                intakeFlag = 0;
            } else if (intakeFlag == 1) {
                intake.setPower(0);
                hold.setPosition(0.3);
                intakeFlag = 0;
            }
        }

        if (gamepad1.yWasPressed()) {   // gate open
            if (gateFlag == 0) {
                gate.setPosition(0.3);
                light.setPosition(0);
                gateFlag = 1;
            }
            else if (gateFlag == 1) {
                gate.setPosition(0.5);
                light.setPosition(0.64);
                gateFlag = 0;
            }
        }

    }
}
