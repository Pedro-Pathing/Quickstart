package org.firstinspires.ftc.teamcode.meet2;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.utils.LoopTimer;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Objects;

@Autonomous(name = "Meet 2 Auto", group = "Autonomous")
//@Configurable // Panels
@Disabled
@SuppressWarnings("FieldCanBeLocal") // Stop Android Studio from bugging about variables being predefined
public class Meet2Auto extends LinearOpMode {

    //Telemetry Manager
    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    //Pedro Variables
    private Pose currentPose;

    //Timer
    private final ElapsedTime launchTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private final ElapsedTime autoTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private final ElapsedTime ballTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    //Panels Editable Variables
    public static double PPGIntakeX = 15.5;
    public static double PGPIntakeX = 12;
    public static double GPPIntakeX = 12;
    public static double intakeMaxPower = 0.5;
    public static double shootDistance = 52;



    //PedroPathing PathChains
    public Pose startPose;
    public PathChain StartToScore;
    public PathChain TurnToPPG;
    public double ScoreWait1;
    public PathChain ScoreToPPG;
    public PathChain PPGToScore;
    public PathChain TurnToPGP;
    public double ScoreWait2;
    public PathChain ScoreToPGP;
    public PathChain PGPIntake;
    public PathChain PGPToScore;
    public double ScoreWait3;
    public PathChain TurnToGPP;
    public PathChain ScoreToGPP;
    public PathChain GPPIntake;
    public PathChain GPPToScore;
    public double ScoreWait4;
    public PathChain Park;

    //Changing variables
    public int autoState = 0;
    private boolean sequenceFinished = false;
    private boolean autoInitialized = false;

    //April Tag Variables
    private AprilTagProcessor aprilTag;
    private final boolean USE_WEBCAM = true;


    private MotorEx fL, fR, bL, bR, launcher1, launcher2, turret, intake;
    private String DTState="drive", intakeState="idle", turretState="idle", launcherState="idle";
    private boolean DTisReady, intakeisReady, turretisReady, launcherisReady;
    private MotorGroup launcher;
    private ElapsedTime teleTimer;
    private MecanumDrive drive;

    //Team Dependents
    private String team = "blue";
    private double goalID = 20;
    private Pose goalPose;
    private Pose aprilTagPose;

    //Lookup Tables
    private InterpLUT velocityLUT = new InterpLUT(), rangeLUT= new InterpLUT();

    //Launcher Variables
    public static double launchTime = 1.5;
    public static double kp = 1.5;
    public static double ki = 200;
    public static double kd = 0;
    private double launcherTargetVelocity;
    public static double launcherTestSpeed = 0.6;
    public double odoRange = 0;
    public int ballsLaunched = 0;
    public static double launchDelaySeconds = 0.15;
    public static double shooterSpeedGap = 20;
    public double previousVelocity = 0;

    //Intake Variables
    public static double intakePickupSpeed = .9;
    public static double transferLoadSpeed = .75;
    public static double intakeRejectSpeed = -0.5;

    //Turret Variables
    private PIDFController turretPIDF;
    public static double turretTolerance = 1;
    public static double tkP = 0.0025;
    public static double tkI = 0;
    public static double tkD = 0.00005;
    public static double tkSCustom = 0.15;
    public static double errorTotal = 30;
    private boolean targetFound = false;
    private int turretTargetPos;
    private double angleError;
    private double goalOffset;
    private boolean turretAimed = false;
    private boolean turretManualControl = false;

    //The variable to store our instance of the vision portal.

    private VisionPortal visionPortal;


    @Override
    public void runOpMode() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // Panels telemetry
        LoopTimer timer = new LoopTimer();
        initMotors(); //Initializes subsystem motors
        createLUTs(); //Initialize lookup tables

        //Initialize PP Follower
        follower= Constants.createFollower(hardwareMap);
        while (opModeInInit()) {
            telemetry.addLine("Driver: Press left stick for blue team and right stick for red team.");
            telemetry.addData("Team Selected:", team);
            telemetry.update();
            if (gamepad1.left_stick_button) {
                team = "blue";
            } else if (gamepad1.right_stick_button) {
                team = "red";
            }
        }
        updateTeamDependents();
        follower.setStartingPose(startPose);


        buildPaths(); //Initialize all Pedro Paths

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL)); //Bulk read to reduce loop time

        launchTimer.reset();
        autoTimer.reset();
        while (opModeIsActive()) {

            hubs.forEach(LynxModule::clearBulkCache); //Bulk reading

            //Update Pedro follower and Panels
            follower.update();
            currentPose = follower.getPose();


            autoStateMachine(); //Overall state machine function to simplify code

            auto(); //All hardware

//            telemetryM.addData("Elapsed",runtime.toString());
            telemetryM.addData("X",currentPose.getX());
            telemetryM.addData("Y",currentPose.getY());
            telemetryM.addData("Heading",currentPose.getHeading());
            //telemetryM.addData("Auto State",autoState);
            //telemetryM.addData("Launcher State",launcherState);
            //telemetryM.addData("Launcher Timer", launchTimer);
            telemetryM.addData("Launcher Velocitu",launcher.getVelocity());
            telemetryM.addData("Turret Target",turretTargetPos);
            telemetryM.addData("Turret Pos",turret.getCurrentPosition());
            telemetryM.addData("Balls Launched",ballsLaunched);
            telemetryM.addData("BallTimer",ballTimer);
            telemetryM.update();
        }
    }


    public void initMotors() {
        fL = new MotorEx(hardwareMap, "fL", Motor.GoBILDA.RPM_312);
        fR = new MotorEx(hardwareMap, "fR", Motor.GoBILDA.RPM_312);
        bL = new MotorEx(hardwareMap, "bL", Motor.GoBILDA.RPM_312);
        bR = new MotorEx(hardwareMap, "bR", Motor.GoBILDA.RPM_312);
        launcher1 = new MotorEx(hardwareMap, "launcherMotor1", Motor.GoBILDA.BARE);
        launcher2 = new MotorEx(hardwareMap, "launcherMotor2", Motor.GoBILDA.BARE);
        intake = new MotorEx(hardwareMap,"intakeMotor",Motor.GoBILDA.BARE);
        turret = new MotorEx(hardwareMap,"turretMotor",Motor.GoBILDA.RPM_435);
        turret.resetEncoder();
        launcher2.setInverted(true);
        launcher1.setVeloCoefficients(kp,ki,kd);
        launcher2.setVeloCoefficients(kp,ki,kd);
        launcher1.setRunMode(Motor.RunMode.VelocityControl);
        launcher2.setRunMode(Motor.RunMode.VelocityControl);
        drive = new MecanumDrive(fL, fR, bL, bR);
        drive.setRightSideInverted(false);
        turretPIDF = new PIDFController(tkP, tkI, tkD, 0);
        turretPIDF.setIntegrationBounds(-errorTotal,errorTotal);
        launcher = new MotorGroup(launcher1, launcher2);
    }

    public void auto() {
        DTAuto();
        turretAuto();
        launcherAuto();
        intakeAuto();
    }

    public void DTAuto() {

        switch (DTState){
            case "idle":
                DTisReady = true;
                break;
            case "drive":
                break;
        }
    }
    public void intakeAuto() {
        switch (intakeState){
            case "idle":
                intakeisReady = true;
                intake.set(0);
                break;
            case "firing":
                intake.set(transferLoadSpeed);
                break;
            case "intaking":
                intake.set(intakePickupSpeed);
                break;
            case "rejecting":
                intake.set(intakeRejectSpeed);
                break;
        }
    }
    public void turretAuto() {

        switch (turretState){
                case "idle":
                    turretisReady = true;
                    targetFound = false;
                    break;
                case "tracking": //track goal whenever possible to save on cycle time, or retain a specific angle
                    turretTrackProcedure();
                    turretisReady = true;
                    targetFound = false;
                    break;
                case "aiming": //Use apriltag to exactly aim at goal and find range to pass into launcher
                    turretisReady = false;
                    turretTargetProcedure();
                    if (Math.abs(angleError) <= 3) {
                        turretState = "aimed";
                    }
                    break;
                case "aimed":
                    turretTargetProcedure();
                    if (Math.abs(angleError) >= 3) {
                        turretState = "aiming";
                    }
            }
        turretControlLoop();
    }
    public void launcherAuto() {
        double botX = currentPose.getX();
        double botY = currentPose.getY();
        double goalX = goalPose.getX();
        double goalY = goalPose.getY();
        switch (launcherState){
            case "idle":
                launcherisReady = true;
                launcher.stopMotor();
                odoRange = Math.hypot(goalX-botX,goalY-botY);
                break;
            case "testSpeed":
                launcherTargetVelocity = launcherTestSpeed;
                launcher.set(launcherTargetVelocity);
                break;
            case "beginLaunchSequence":
                launcherSpinUp();
                launcher.set(launcherTargetVelocity);
                launcherisReady = false;
                launcherState = "aiming";
                turretState = "aiming";
                break;
            case "aiming":
                launcherSpinUp();
                launcher.set(launcherTargetVelocity);
                intakeState = "idle";
                if (Math.abs(launcher.getVelocity()) > Math.abs(velocityLUT.get(launcherTargetVelocity)) - shooterSpeedGap) {
                    launcherState = "acc_ready";
                }
                break;
            case "acc_ready":
                launcherSpinUp();
                launcher.set(launcherTargetVelocity);
                if (Objects.equals(turretState, "aimed")) {
                    intakeState = "firing";
                    launcherState = "firing";
                    break;
                }
            case "firing":
                launcherSpinUp();
                launcher.set(launcherTargetVelocity);
                if (Objects.equals(turretState,"aiming")) {
                    intakeState = "idle";
                    launcherState = "aiming";
                }
                break;
            case "preparing":
                launcherSpinUp();
                launcher.set(launcherTargetVelocity);
                break;
            case "rejecting":
                launcherTargetVelocity = -.3;
                launcher.set(launcherTargetVelocity);
                launcherisReady=true;
                break;
        }
    }


    public void createLUTs() {
        //create shooting speed lookup table

        //Add values (obtained empirically)
        //Input is distance, output is shooter velocity
        rangeLUT.add(20,0.43);
        rangeLUT.add(48,0.43);
        rangeLUT.add(55,0.43);
        rangeLUT.add(65.7,0.465);
        rangeLUT.add(75,0.48);
        rangeLUT.add(85,0.515);
        rangeLUT.add(100,.535);
        rangeLUT.createLUT();

        velocityLUT.add(-1,2500);
        velocityLUT.add(-0.8,2000);
        velocityLUT.add(-0.6,1500);
        velocityLUT.add(-.4,1000);
        velocityLUT.add(-.2,500);
        velocityLUT.add(0,0);
        velocityLUT.add(0.2,500);
        velocityLUT.add(0.4,1000);
        velocityLUT.add(0.6,1500);
        velocityLUT.add(0.8,2000);
        velocityLUT.add(1,2500);
        velocityLUT.createLUT();
    }

    public double calculateRangeLUT(double input) {
        if (input < 20 || input > 100) {
            return 0.45;
        } else {
            return rangeLUT.get(input);
        }
    }

    public void launcherSpinUp() {
        double botX = currentPose.getX();
        double botY = currentPose.getY();
        double goalX = goalPose.getX();
        double goalY = goalPose.getY();
        odoRange = Math.hypot(goalX-botX,goalY-botY);
        launcherTargetVelocity = calculateRangeLUT(odoRange);
    }

    public void turretTargetProcedure() {
        turretTrackProcedure();
        angleError = turretTicksToAngle(turretTargetPos-turret.getCurrentPosition());
    }
    public void turretTrackProcedure() {
        double turretAngle;
        turretAngle = calculateTurretAngle(currentPose.getX(), currentPose.getY(), Math.toDegrees((currentPose.getHeading())));
        turretAngle = turretAngleLimiter(turretAngle);
        turretTargetPos = turretAngleToTicks(turretAngle);
    }

    public double calculateTurretAngle(double botX, double botY, double botHeading) {
        double goalX = goalPose.getX();
        double goalY = goalPose.getY();
        double targetAngle = Math.toDegrees(Math.atan2(goalY-botY,goalX-botX));
        targetAngle -= botHeading;
        telemetryM.addData("Target Angle",targetAngle);
        return targetAngle;
    }

    public void turretControlLoop() {
        double output = turretPIDF.calculate(turret.getCurrentPosition(),turretTargetPos);
        double kSFriction;
        if (Math.abs(turretPIDF.getPositionError()) <= turretTolerance) kSFriction = 0;
        else {
            kSFriction = tkSCustom * (Math.abs(turretPIDF.getPositionError()) / turretPIDF.getPositionError());
        }
        output += kSFriction;
        turret.set(output);
    }

    public int turretAngleToTicks(double angle) {
        return (int) (angle * 978.7 / 360);
    }

    public int turretTicksToAngle(double ticks) {
        return (int) (ticks * 360 / 978.7);
    }

    public double turretAngleLimiter(double angleAttempt) {
        double realAngle = angleAttempt;
        if (realAngle > 180) {
            realAngle -= 360;
        } else if (realAngle < -180) {
            realAngle += 360;
        }
        if (Math.abs(realAngle) > 170) {
            return turretTicksToAngle(turretTargetPos);
        }
        if (realAngle > 150) {
            realAngle = 150;
        } else if (realAngle < -150) {
            realAngle = -150;
        }
        return realAngle;
    }

    public void updateTeamDependents() {
        if (Objects.equals(team,"blue")){
            goalID = 20;
            goalOffset = 135;
            startPose = new Pose(29.5,134,Math.toRadians(90)); // placeholder
            goalPose = new Pose(0,144,Math.toRadians(135));
            aprilTagPose = new Pose(15,130,0);
        } else if (Objects.equals(team,"red")) {
            goalID = 24;
            goalOffset = 45;
            startPose = new Pose(114.5,134,Math.toRadians(90)); // placeholder
            goalPose = new Pose(144,144,Math.toRadians(45));
            aprilTagPose = new Pose(129,130,0);
        }
    }

    public double x(double X) { //transform X based on team
        if (Objects.equals(team, "blue")) return X;
        else return 144-(-1*X);
    }

    public double a(double angle) { //transform heading based on team
        if (Objects.equals(team,"blue")) return Math.toRadians(angle);
        else return Math.toRadians(-(angle-90)+90);
    }

    Runnable activateTurret = new Runnable() {
        @Override
        public void run() {
            turretState = "idle";
            launcherState = "preparing";
        }
    };
    private void autoStateMachine() {
        double turretAngle;
        switch (autoState) {
            case -1:
                break;
            case 0:
                follower.followPath(StartToScore); // Move to scoring position
                autoState=1;
                turretAngle = calculateTurretAngle(StartToScore.endPoint().getX(), StartToScore.endPoint().getY(), 135);
                turretAngle = turretAngleLimiter(turretAngle);
                turretTargetPos = turretAngleToTicks(turretAngle);
                break;
            case 1:
                if (!follower.isBusy()) { //Once move is finished
                    launcherState = "beginLaunchSequence"; //launch procedure
                    turretState = "aiming";
                    intakeState = "idle";
                    launchTimer.reset();

                    autoState=2;

                }
                break;
            case 2:
                if (launchTimer.seconds() > launchTime || launcherisReady) { //Once artifacts are scored and not turning
                    turretState = "idle";
                    launcherState = "rejecting"; //intake mode
                    intakeState = "intaking";
                    follower.turnTo(a(180));
                    ballsLaunched=0;
                    autoState = -10;
                }
                break;
            case -10: //don't feel like changing every number
                if (!follower.isBusy()) {
                    follower.followPath(ScoreToPPG,intakeMaxPower,true); // Move to intake PPG
                    autoState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) { //Once move is finished
                    intakeState = "idle";
                    launcherState = "rejecting";
                    follower.followPath(PPGToScore); // Move to score position
                    autoState=4;
                    launchTimer.reset();
                    turretAngle = calculateTurretAngle(PPGToScore.endPoint().getX(), PPGToScore.endPoint().getY(), 180);
                    turretAngle = turretAngleLimiter(turretAngle);
                    turretTargetPos = turretAngleToTicks(turretAngle);

                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    launcherState = "beginLaunchSequence"; //launch procedure
                    turretState = "aiming";
                    intakeState = "idle";
                    launchTimer.reset();
                    autoState=5;
                }
                break;
            case 5:
                if (launchTimer.seconds()>launchTime || launcherisReady) { //Once launching is (hopefully) finished
                    launcherState = "idle";
                    turretState = "idle";
                    intakeState = "idle";
                    ballsLaunched=0;
                    follower.turnTo(a(256));
                    autoState=6;
                }
                break;
            case 6:
                if (!follower.isBusy()) { //If turn is finished
                    follower.followPath(ScoreToPGP); // Move to PGP intake position
                    autoState=7;
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    intakeState = "intaking"; // Activate intake
                    launcherState = "rejecting";
                    follower.followPath(PGPIntake,intakeMaxPower,true); // Move forwards while intaking
                    autoState=8;
                }
                break;
            case 8:
                if (!follower.isBusy()) { // Once move is complete
                    intakeState = "idle"; // Disable intake/launcher
                    launcherState = "rejecting"; // Run launcher backwards to push out stuck artifacts
                    follower.followPath(PGPToScore); // Go back to scoring position
                    autoState=9;
                    turretAngle = calculateTurretAngle(PGPToScore.endPoint().getX(), PGPToScore.endPoint().getY(), 225);
                    turretAngle = turretAngleLimiter(turretAngle);
                    turretTargetPos = turretAngleToTicks(turretAngle);
                }
                break;
            case 9:
                if (!follower.isBusy()) { // Once move is complete
                    launcherState = "beginLaunchSequence"; //launch procedure
                    turretState = "aiming";
                    intakeState = "idle";
                    launchTimer.reset();
                    autoState=-100;
                }
                break;
            case -100: //don't want to disrupt order
                if (!follower.isBusy() && ( launchTimer.seconds() > launchTime || launcherisReady)){
                    ballsLaunched=0;
                    follower.turnTo(a(270));
                    autoState = 10;
                }
                break;
            case 10:
                if (!follower.isBusy()) { //If shooter is finished
                    launcherState = "idle";
                    turretState = "idle";
                    intakeState = "idle";
                    follower.followPath(ScoreToGPP,1.0,true); // Go to GPP intake position
                    autoState=11;
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    intakeState = "intaking"; // Activate intake
                    launcherState = "rejecting";
                    follower.followPath(GPPIntake,intakeMaxPower,true); // Move forwards while intaking
                    autoState=12;
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    intakeState = "idle"; // Disable intake/launcher
                    launcherState = "rejecting"; // Run launcher backwards to push out stuck artifacts
                    follower.followPath(GPPToScore); // Go back to scoring position
                    autoState = -1000;
                    turretAngle = calculateTurretAngle(GPPToScore.endPoint().getX(), GPPToScore.endPoint().getY(), 240);
                    turretAngle = turretAngleLimiter(turretAngle);
                    turretTargetPos = turretAngleToTicks(turretAngle);
                }
                break;
            case -1000:
                if (!follower.isBusy()) {
                    follower.turnTo(a(240));
                    autoState = 13;
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    launcherState = "beginLaunchSequence"; //launch procedure
                    turretState = "aiming";
                    intakeState = "idle";
                    launchTimer.reset();
                    autoState = -3;
                }
                break;
            case -3:
                if (launchTimer.seconds() > launchTime || launcherisReady) {
                    follower.turnTo(a(258));
                    autoState = 14;
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    launcherState = "idle";
                    turretState = "idle";
                    intakeState = "idle";
                    ballsLaunched=0;
                    autoState = 15;
                }
                break;
            case 15:
                follower.followPath(Park);
                autoState = 16;
                break;
            case 16: //resetting or any other things necessary before autonomous ends
                if (!follower.isBusy()) {
                    intakeState = "idle";
                    turretState = "idle";
                    launcherState = "idle";
                    autoState=-1;
                }
                break;
        }
    }

    public void buildPaths() {


        StartToScore = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(startPose.getY(), startPose.getX()),
                                new Pose(x(29.5), 106.5),
                                new Pose(x(52), 84)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addParametricCallback(.25, activateTurret)
                .build();
        TurnToPPG = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(52), 84), new Pose(x(52), 84))
                )
                .setLinearHeadingInterpolation(a(135), a(180))
                .build();

        ScoreWait1 = 1000;

        ScoreToPPG = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(52), 84), new Pose(x(16), 84))
                )
                .setTangentHeadingInterpolation()
                .build();

        PPGToScore = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(16), 84), new Pose(x(58), 84))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addParametricCallback(.25,activateTurret)
                .build();

        TurnToPGP = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(58), 84), new Pose(x(58), 84))
                )
                .setLinearHeadingInterpolation(a(180), a(256))
                .build();

        ScoreWait2 = 1000;

        ScoreToPGP = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(x(58), 84),
                                new Pose(x(52), 60),
                                new Pose(x(43), 60)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        PGPIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(43), 60), new Pose(x(16), 60))
                )
                .setTangentHeadingInterpolation()
                .build();

        PGPToScore = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(x(16), 60),
                                new Pose(x(46), 60),
                                new Pose(x(60), 84)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addParametricCallback(.25,activateTurret)
                .build();

        ScoreWait3 = 1000;

        TurnToGPP = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(60), 84), new Pose(x(60), 84))
                )
                .setLinearHeadingInterpolation(a(240), a(270))
                .build();

        ScoreToGPP = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(x(60), 84),
                                new Pose(x(59), 36),
                                new Pose(x(43), 36)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        GPPIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(43), 36), new Pose(x(10), 36))
                )
                .setTangentHeadingInterpolation()
                .build();

        GPPToScore = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(x(10), 36),
                                new Pose(x(50), 36),
                                new Pose(x(60), 84)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addParametricCallback(.25,activateTurret)
                .build();

        ScoreWait3 = 1000;

        Park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(60), 84), new Pose(x(56), 65.8))
                )
                .setTangentHeadingInterpolation()
                .build();

        }

}

