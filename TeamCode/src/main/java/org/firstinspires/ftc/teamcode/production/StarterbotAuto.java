package org.firstinspires.ftc.teamcode.production;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "StarterBot Teleop", group = "StarterBot")
//@Disabled
public class StarterbotAuto extends OpMode {

    // The vast majority of this code will be ripped straight out of the StarterbotTeleop
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotorEx launcher = null;

    public final int LAUNCHER_TARGET_VELOCITY = 1250;
    public final int LAUNCHER_MIN_VELOCITY = 1200;

    private enum LauncherStates {
        IDLE,
        SPIN_UP,
        LAUNCHING,
        DONE,
    }
    private LauncherStates launcherState = LauncherStates.IDLE;

    private final ElapsedTime shotTimer = new ElapsedTime();
    public final double SHOT_TIME = 4.5;

    private CRServo windmillServo = null;

    public Follower follower; // Establishes the follower
    private final PoseFactory poseFactory = PoseFactory.degrees(); // Making the poseFactory which is needed for pedro pathing

    private final Pose start = poseFactory.of(55, 9, 90);
    private final Pose path1 = poseFactory.of(9, 96, 90);
    private final Pose path1Control1 = poseFactory.of(23, 15, 0);

    public Path path1() {
        return Paths.curve(start, path1Control1, path1).linear(start, path1);
    }

    private enum AutoStates { // So this auto just launches, moves at the specific angle and then stops in the parking zone.
        LAUNCH,
        LAUNCHING,
        MOVE,
        REST
    }

    private AutoStates autoState = AutoStates.LAUNCH;

    @Override
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");

        leftFrontDrive.setDirection(StarterbotTeleop.driveDirections[1]);
        rightFrontDrive.setDirection(StarterbotTeleop.driveDirections[0]);
        leftBackDrive.setDirection(StarterbotTeleop.driveDirections[3]);
        rightBackDrive.setDirection(StarterbotTeleop.driveDirections[2]);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);

        follower = Constants.create(hardwareMap);
        follower.setPose(start);
    }

    @Override
    public void loop() {
        switch (autoState) {
            case LAUNCH:
                launch(true);
                autoState = AutoStates.LAUNCHING;
                break;
            case LAUNCHING:
                if (launch(false)) {
                    autoState = AutoStates.MOVE;
                    follower.follow(path1());
                }
                break;
            case MOVE:
                follower.update();
                if (!follower.isBusy()) {
                    autoState = AutoStates.REST;
                }
                break;
            case REST:
                rightFrontDrive.setPower(0);
                leftFrontDrive.setPower(0);
                rightBackDrive.setPower(0);
                leftBackDrive.setPower(0);


        }
    }

    @Override
    public void stop() {

        rightFrontDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);


    }

    private boolean launch(boolean shotRequested) {
        switch (launcherState){
            case IDLE:
                if (shotRequested) launcherState = LauncherStates.SPIN_UP;
                break;
            case SPIN_UP:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    shotTimer.reset();
                    launcherState = LauncherStates.LAUNCHING;
                }
                break;
            case LAUNCHING:
                windmillServo.setPower(1);
                if (shotTimer.seconds() > SHOT_TIME) launcherState = LauncherStates.DONE;
                break;
            case DONE:
                windmillServo.setPower(0);
                launcherState = LauncherStates.IDLE;
                return true;
        }
        return false;
    }
}