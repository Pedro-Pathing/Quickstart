package org.firstinspires.ftc.teamcode.Autos;
import static android.os.SystemClock.sleep;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

@Autonomous
@Configurable // Panels
public class AutoBottomBlue extends OpMode {

    private DcMotor intake;
    private Servo gate, flick, hold;  // servos
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private ShooterSubsystem shooter;
    private double lastTime = 0.0;
    private boolean shooterActive = false;
    private int launchState3 = 0;
    private int launchState2 = 0;
    private int launchState1 = 0;
    private Timer pathTimer;
    private Timer launchTimer;
    private boolean isDone;



    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(55.087885985748215, 9.254156769596198, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths
        pathTimer = new Timer();
        launchTimer = new Timer();
        intake = hardwareMap.get(DcMotor.class, "intake");
        gate = hardwareMap.get(Servo.class, "gate");
        hold = hardwareMap.get(Servo.class, "hold");
        flick = hardwareMap.get(Servo.class,"flick");
        gate.setPosition(0.3);
// Shooter subsystem
        shooter = new ShooterSubsystem(hardwareMap);
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }
    @Override
    public void start() {
        lastTime = getRuntime();
        shooter.getLimelight().start();
        shooter.getLimelight().pipelineSwitch(1); // 1 is for blue tracking
    }
    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // launcher update
        if (shooterActive) {
            double currentTime = getRuntime();
            double dt = currentTime - lastTime;
            lastTime = currentTime;

            shooter.update(follower.getPose(), follower.getVelocity(), dt, telemetry);
        }

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());

        telemetry.update();
        panelsTelemetry.update(telemetry);
    }

    private void launch3balls() {  // we call this function every time you want to launch 3 balls
        switch (launchState3) {
            case 0:
                gate.setPosition(0.55);  //0.55 open 0.3 closed
                launchTimer.resetTimer();
                launchState3++;
                hold.setPosition(0.5);
                break;

            case 1:
                if (launchTimer.getElapsedTimeSeconds() > 1) {
                    intake.setPower(1);
                    launchTimer.resetTimer();
                    launchState3++;
                }
                break;
            case 2:
                intake.setPower(1);
                if (launchTimer.getElapsedTimeSeconds() > 1) {
                    flick.setPosition(0.6);  //0 is up
                    sleep(300);
                    flick.setPosition(1);
                    launchTimer.resetTimer();
                    isDone = true;
                }
                break;
        }
    }

    public static class Paths {
        public PathChain Starttoshoot1;
        public PathChain Shoot1topickup1;
        public PathChain Pickup1tointake1;
        public PathChain Intake1toshoot2;
        public PathChain Shoot2topark;

        public Paths(Follower follower) {
            Starttoshoot1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(55.088, 9.254),

                                    new Pose(61.017, 23.914)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))

                    .build();

            Shoot1topickup1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(61.017, 23.914),
                                    new Pose(30.785, 8.586),
                                    new Pose(15.562, 8.216)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-180))

                    .build();

            Pickup1tointake1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.562, 8.216),

                                    new Pose(11.135878430586969, 8.164224594339366)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Intake1toshoot2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(11.135878430586969, 8.164224594339366),
                                    new Pose(33.806, 21.343),
                                    new Pose(61.071, 23.919)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(90))

                    .build();

            Shoot2topark = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(61.071, 23.919),
                                    new Pose(48.837, 13.333),
                                    new Pose(35.340, 11.164)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-180))

                    .build();
        }
    }

    /* You could check for
                - Follower State: "if(!follower.isBusy()) {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */
    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                shooterActive = !shooterActive;

                follower.followPath(paths.Starttoshoot1,true);
                setPathState(1);
                break;

            case 1:

                if (pathTimer.getElapsedTimeSeconds() > 4) {
                    launch3balls();
                }

                if (isDone) {
                    isDone = false;
                    launchState3 = 0;
                    launchState2 = 0;
                    launchState1 = 0;
                    gate.setPosition(0.3);
                    follower.followPath(paths.Shoot1topickup1, true);
                    setPathState(2);

                }
                break;

            case 2:

                if (!follower.isBusy()) {
                    follower.followPath(paths.Pickup1tointake1, 0.7,true);
                    setPathState(3);
                }
                break;

            case 3:

                if (!follower.isBusy()) {
                    intake.setPower(0);
                    follower.followPath(paths.Intake1toshoot2, true);
                    setPathState(4);
                }
                break;

            case 4:

                if (pathTimer.getElapsedTimeSeconds() > 4) {
                    launch3balls();
                }

                if (isDone) {
                    isDone = false;
                    launchState3 = 0;
                    launchState2 = 0;
                    launchState1 = 0;

                    follower.followPath(paths.Shoot2topark, true);
                    setPathState(-1);
                }
                break;

        }
        return pathState;
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}