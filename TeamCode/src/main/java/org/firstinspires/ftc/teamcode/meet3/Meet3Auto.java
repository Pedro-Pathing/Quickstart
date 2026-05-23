package org.firstinspires.ftc.teamcode.meet3;

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

import java.io.FileNotFoundException;
import java.util.List;
import java.util.Objects;


@Autonomous(name = "Meet 3 Auto", group = "Autonomous")
//@Configurable // Panels
@Disabled
@SuppressWarnings("FieldCanBeLocal") // Stop Android Studio from bugging about variables being predefined
public class Meet3Auto extends LinearOpMode {

    //Telemetry Manager
    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    //Pedro Variables
    private Pose currentPose;

    //Timer
    private final ElapsedTime launchTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private final ElapsedTime autoTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private final ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    //Panels Editable Variables
    public static double intakeMaxPower = 1.0;
    public static double rampMaxPower = 1;
    public static double gateWaitTime = 1.4;
    public static double launchTime15 = 1.15;
    public static double gateWait15 = 1.4;
    public static double intakeMaxPower15 = 0.7;
    public static double rampMaxPower15 = 0.6;
    public static double farLaunchTime = 2.5;
    public static Pose endPosition = new Pose(0,0,0);

    public double selectedAuto = 15; //default 15 ball



    //PedroPathing PathChains
    public Pose startPose;
    public PathChain startToScore;
    public PathChain scoreToPPG;
    public PathChain PPGIntake;
    public PathChain PPGToRamp;
    public PathChain rampToScore;
    public PathChain scoreToPGP;
    public PathChain PGPIntake;
    public PathChain PGPToScore;
    public PathChain scoreToGPP;
    public PathChain GPPIntake;
    public PathChain GPPToScore;
    public PathChain Park;
    public PathChain StartToScore;
    public PathChain ScoreToPGP18;
    public PathChain PGPIntake18;
    public PathChain PGPToScore18;
    public PathChain ScoreToRamp1;
    public PathChain ScoreToRamp2;
    public PathChain IntakeRamp;
    public PathChain RampToScore1;
    public PathChain RampToScore2;
    public PathChain PPGIntake18;
    public PathChain PPGToScore;
    public PathChain ScoreToGPP;
    public PathChain GPPIntake18;
    public PathChain GPPToScore18one;
    public PathChain GPPToScore18two;
    public PathChain Park18;
    public PathChain startToScore15;
    public PathChain scoreToPGP15;
    public PathChain PGPIntake15;
    public PathChain PGPToScore15;
    public PathChain ScoreToRamp115;
    public PathChain ScoreToRamp215;
    public PathChain IntakeRamp15;
    public PathChain RampToScore115;
    public PathChain RampToScore215;
    public PathChain PPGIntake15;
    public PathChain PPGToScore15;
    public PathChain ScoreToGPP15;
    public PathChain GPPIntake15;
    public PathChain GPPToScore15one;
    public PathChain Park15;
    public PathChain farScoreToGPP;
    public PathChain farGPPIntake;
    public PathChain farGPPIntakeToScore;
    public PathChain farScoreToCorner;
    public PathChain cornerIntake;
    public PathChain cornerToFarScore;
    public PathChain farPark;

    //Changing variables
    public int autoState = 0;

    private MotorEx fL, fR, bL, bR, launcher1, launcher2, turret, intake;
    private String DTState="drive", intakeState="idle", turretState="idle", launcherState="idle";
    private boolean DTisReady, intakeisReady, turretisReady, launcherisReady;
    private MotorGroup launcher;
    private MecanumDrive drive;

    //Team Dependents
    public static String team = "blue";
    private double goalID = 20;
    private Pose goalPose;
    private Pose aprilTagPose;

    //Lookup Tables
    private InterpLUT velocityLUT = new InterpLUT(), rangeLUT= new InterpLUT();

    //Launcher Variables
    public static double launchTime = .85;
    public static double kp = 1.5;
    public static double ki = 200;
    public static double kd = 0;
    private double launcherTargetVelocity;
    public static double launcherTestSpeed = 0.6;
    public double odoRange = 0;
    public static double shooterSpeedGap = 40;
    public double preparedLauncherVelocity = 0;

    //Intake Variables
    public static double intakePickupSpeed = 1.0;
    public static double transferLoadSpeed = 1.0;
    public static double intakeRejectSpeed = -0.9;

    //Turret Variables
    private PIDFController turretPIDF;
    public static double turretTolerance = 1;
    public static double tkP = 0.0025;
    public static double tkI = 0;
    public static double tkD = .00005;
    public static double tkSCustom = 0.15;
    public static double errorTotal = 30;
    private int turretTargetPos;
    private double angleError;
    public int preparedTargetPos =0;
    private double backLashOffset = 0;



    @Override
    public void runOpMode() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // Panels telemetry
        LoopTimer timer = new LoopTimer();
        initMotors(); //Initializes subsystem motors
        createLUTs(); //Initialize lookup tables

        //Initialize PP Follower
        follower= Constants.createFollower(hardwareMap);
        while (opModeInInit()) {
            telemetry.addLine("Driver: Press left stick for blue team and right stick for red team");
            telemetry.addLine("A for 12 ball, X for 15 ball, B for 18 ball, Y for far zone (9)");
            telemetry.addData("Team Selected:", team);
            telemetry.addData("Auto Selected:",selectedAuto);
            telemetry.update();
            if (gamepad1.left_stick_button) {
                team = "blue";
            } else if (gamepad1.right_stick_button) {
                team = "red";
            }
            if (gamepad1.a) selectedAuto = 12;
            else if (gamepad1.x) selectedAuto = 15;
            else if (gamepad1.b) selectedAuto = 18;
            else if (gamepad1.y) selectedAuto = 9;
        }
        updateTeamDependents();
        follower.setStartingPose(startPose);


        if (selectedAuto == 12) buildPaths12(); //Initialize all Pedro Paths
        if (selectedAuto == 15) buildPaths15();
        if (selectedAuto == 18) buildPaths18();
        if (selectedAuto == 9) buildPaths9();

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL)); //Bulk read to reduce loop time

        launchTimer.reset();
        autoTimer.reset();
        while (opModeIsActive()) {

            hubs.forEach(LynxModule::clearBulkCache); //Bulk reading

            //Update Pedro follower and Panels
            follower.update();
            currentPose = follower.getPose();


            if (selectedAuto == 12) autoStateMachine12(); //Overall state machine function to simplify code
            if (selectedAuto == 15) autoStateMachine15();
            if (selectedAuto == 18) autoStateMachine18();
            if (selectedAuto == 9) autoStateMachine9();

            auto(); //All subsystems

            updateTelemetry();
            telemetryM.update(telemetry);
        }
        endPosition = follower.getPose();
    }

    private void updateTelemetry() {
        telemetryM.addData("Auto Timer",autoTimer.seconds());
        telemetryM.addLine("Robot Position");
        telemetryM.addData("X",currentPose.getX());
        telemetryM.addData("Y",currentPose.getY());
        telemetryM.addData("Heading",Math.toDegrees(currentPose.getHeading()));
        telemetryM.addData("Goal Distance",odoRange);
        telemetryM.addLine("Launcher Info");
        telemetryM.addData("Launcher Velocity",launcher.getVelocity());
        telemetryM.addLine("Turret Info");
        telemetryM.addData("Turret Target",turretTargetPos);
        telemetryM.addData("Turret Pos",turret.getCurrentPosition());
        telemetryM.addData("Turret Angle",turret.getCurrentPosition() * 360/978.7);
        telemetryM.addData("Turret Motor Power",turret.get());
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
        launcher1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        launcher2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
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
                if (Math.abs(intake.getVelocity()) < 300) intake.set(1.0);
                else intake.set(transferLoadSpeed);
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
                break;
            case "tracking": //track goal whenever possible to save on cycle time, or retain a specific angle
                turretTrackProcedure();
                turretisReady = true;
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
                break;
            case "preparing":
                turretTargetPos = preparedTargetPos;
                break;
        }
        turretControlLoop();
    }

    public void launcherAuto() {
        double botX = currentPose.getX();
        double botY = currentPose.getY();
        double goalX = goalPose.getX();
        double goalY = goalPose.getY();
        if (Objects.equals(launcherState, "firing")) launcher.setRunMode(Motor.RunMode.RawPower);
        else launcher.setRunMode(Motor.RunMode.VelocityControl);
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
                launchTimer.reset();
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
                if (launcher.getVelocity() > velocityLUT.get(launcherTargetVelocity)) {
                    launcher.set(0);
                } else launcher.set(1);
                //launcher.set(launcherTargetVelocity);
                if (Objects.equals(turretState,"aiming")) {
                    intakeState = "idle";
                    launcherState = "aiming";
                }
                break;
            case "preparing":
                launcher.set(preparedLauncherVelocity);
                break;
            case "rejecting":
                launcherTargetVelocity = -.35;
                launcher.set(launcherTargetVelocity);
                launcherisReady=true;
                break;
            case "rejectFaster":
                launcherTargetVelocity = -.6;
                launcher.set(launcherTargetVelocity);
                break;
        }
    }

    public void createLUTs() {
        //create shooting speed lookup table

        //Add values (obtained empirically)
        //Input is distance, output is shooter velocity
//        rangeLUT.add(20,0.43);
//        rangeLUT.add(48,0.43);
//        rangeLUT.add(55,0.43);
//        rangeLUT.add(65.7,0.445);
//        rangeLUT.add(75,0.47);
//        rangeLUT.add(85,0.495);
//        rangeLUT.add(100,.53);
//        rangeLUT.add(110,0.55);
//        rangeLUT.add(115,0.57);
//        rangeLUT.add(123,0.67);
//        rangeLUT.add(130,0.67);
//        rangeLUT.add(140,0.74);
//        rangeLUT.add(160,0.79);
        rangeLUT.add(20,0.46);
        rangeLUT.add(48,0.46);
        rangeLUT.add(55,0.46);
        rangeLUT.add(65.7,0.48);
        rangeLUT.add(75,0.5);
        rangeLUT.add(85,0.54);
        rangeLUT.add(100,.57);
        rangeLUT.add(110,0.6);
        rangeLUT.add(115,0.62);
        rangeLUT.add(123,0.69);
        rangeLUT.add(130,0.69);
        rangeLUT.add(140,0.76);
        rangeLUT.add(160,0.81);
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
        if (input < 20 || input > 160) {
            if (input > 110) transferLoadSpeed = 0.6;
            else transferLoadSpeed = 1.0;
            return launcherTestSpeed;
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
        targetAngle -= botHeading + 180;
        telemetryM.addData("Target Angle",targetAngle);
        return targetAngle;
    }

    public void turretControlLoop() {
        double output = turretPIDF.calculate(turret.getCurrentPosition(),turretTargetPos);
        double kSFriction;
        if (Math.abs(turretPIDF.getPositionError()) <= turretTolerance) {
            kSFriction = 0;
            turret.stopMotor();
        }
        else {
            kSFriction = tkSCustom * (Math.abs(turretPIDF.getPositionError()) / turretPIDF.getPositionError());
            output += kSFriction;
            turret.set(output);
        }
    }

    public void premoveTurret(double x, double y,double heading) {
        double turretAngle;
        turretAngle = calculateTurretAngle(x, y,heading);
        turretAngle = turretAngleLimiter(turretAngle);
        preparedTargetPos = turretAngleToTicks(turretAngle);
        double goalX = goalPose.getX();
        double goalY = goalPose.getY();
        odoRange = Math.hypot(goalX-x,goalY-y);
        preparedLauncherVelocity = calculateRangeLUT(odoRange);
    }

    public int turretAngleToTicks(double angle) {
        return (int) (angle * 1076.6 / 360); //978.7 with 435
    }

    public int turretTicksToAngle(double ticks) {
        return (int) (ticks * 360 / 1076.6); //978.7 with 435
    }

    public double turretAngleLimiter(double angleAttempt) {
        double realAngle = angleAttempt;
        if (realAngle > 180) {
            realAngle -= 360;
        } else if (realAngle < -180) {
            realAngle += 360;
        }
        if ((realAngle > 160 && turret.getCurrentPosition() < 0) || (realAngle < -160 && turret.getCurrentPosition() > 0)) {
            return turretTicksToAngle(turretTargetPos);
        }
        if (realAngle > 160) {
            realAngle = 160;
        } else if (realAngle < -135) {
            realAngle = -135;
        }
        return realAngle;
    }

    public void updateTeamDependents() {
        if (Objects.equals(team,"blue")){
            goalID = 20;
            if (selectedAuto == 9) startPose = new Pose (54,7,Math.toRadians(90));
            else startPose = new Pose(18.5,118.5,Math.toRadians(143+180));
            goalPose = new Pose(0,144,Math.toRadians(135));
            aprilTagPose = new Pose(15,130,0);
            backLashOffset = 1;
        } else if (Objects.equals(team,"red")) {
            goalID = 24;
            if (selectedAuto == 9) startPose = new Pose(90,7,Math.toRadians(90));
            else startPose = new Pose(x(18.5),118.5,a(143+180));
            goalPose = new Pose(144,144,Math.toRadians(45));
            aprilTagPose = new Pose(129,130,0);
            backLashOffset = -1;
        }
    }

    public double x(double X) { //transform X based on team
        if (Objects.equals(team, "blue")) return X;
        else return 144-(X);
    }

    public double a(double angle) { //transform heading based on team
        if (Objects.equals(team,"blue")) return Math.toRadians(angle);
        else return Math.toRadians(-(angle-90)+90);
    }

    public void resetSubsystems() {
        launcherState = "idle";
        intakeState = "idle";
        turretState = "idle";
    }

    public void intakeBalls() {
        launcherState = "rejecting";
        intakeState = "intaking";
    }

    public void launchBalls() {
        launcherState = "beginLaunchSequence";
    }

    Runnable preSpinLauncher = new Runnable() {
        @Override
        public void run() {
            intakeState = "idle";
            launcherState = "preparing";
            turretState = "preparing";
        }
    };

    Runnable stopIntake = new Runnable() {
        @Override
        public void run() {
            intakeState = "idle";
        }
    };

    Runnable rejectIntake = new Runnable() {
        @Override
        public void run() {
            intakeState = "rejecting";
        }
    };

    public void autoStateMachine9() {
        switch (autoState) {
            case -1:
                break;
            case 0:
                transferLoadSpeed = 0.6;
                launchBalls();
                autoState = 1;
                break;
            case 1:
                if (launchTimer.seconds() > farLaunchTime + 1) {
                    resetSubsystems();
                    follower.followPath(farScoreToGPP);
                    autoState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    intakeBalls();
                    follower.followPath(farGPPIntake,intakeMaxPower15,true);
                    autoState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    resetSubsystems();
                    launcherState = "rejecting";
                    intakeState = "intaking";
                    premoveTurret(x(55),12,Math.toDegrees(a(149)));
                    follower.followPath(farGPPIntakeToScore);
                    autoState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    launchBalls();
                    autoState = 5;
                }
                break;
            case 5:
                if (launchTimer.seconds() > farLaunchTime) {
                    resetSubsystems();
                    follower.followPath(farScoreToCorner);
                    autoState = 6;
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    intakeBalls();
                    follower.followPath(cornerIntake,intakeMaxPower15,true);
                    waitTimer.reset();
                    autoState = 7;
                }
                break;
            case 7:
                if (!follower.isBusy() || waitTimer.seconds() > 1.75) {
                    resetSubsystems();
                    launcherState = "rejecting";
                    intakeState = "intaking";
                    premoveTurret(x(54),14,Math.toDegrees(a(180)));
                    follower.followPath(cornerToFarScore);
                    autoState = 8;
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    launchBalls();
                    autoState = 9;
                }
                break;
            case 9:
                if (launchTimer.seconds() > farLaunchTime) {
                    resetSubsystems();
                    follower.followPath(farPark);
                    autoState = 10;
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    autoState = 11;
                }
                break;
            case 11:
                telemetryM.addLine("Autonomous Complete!!! :D");
                endPosition = follower.getPose();
                break;
        }
    }

    public void buildPaths9() {
        farScoreToGPP = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(54), 7), new Pose(x(54), 36))
                )
                .setLinearHeadingInterpolation(a(90), a(180),0.8)
                .build();

        farGPPIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(54), 36), new Pose(x(15), 36))
                )
                .setTangentHeadingInterpolation()
                .build();

        farGPPIntakeToScore = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(15), 36), new Pose(x(55), 12))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addTemporalCallback(200,stopIntake)
                .addTemporalCallback(300, rejectIntake)
                .addTemporalCallback(400, stopIntake)
                .addTemporalCallback(500, preSpinLauncher)
                .build();

        farScoreToCorner = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(55), 12), new Pose(x(10), 33))
                )
                .setLinearHeadingInterpolation(a(145), a(270),0.8)
                .build();

        cornerIntake = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(x(7), 36), new Pose(x(7), 13)))
                .setTangentHeadingInterpolation()
                .build();

        cornerToFarScore = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(7), 13), new Pose(x(54), 14))
                )
                .setLinearHeadingInterpolation(a(270), a(180),0.8)
                .addTemporalCallback(200,stopIntake)
                .addTemporalCallback(300, rejectIntake)
                .addTemporalCallback(400, stopIntake)
                .addTemporalCallback(500, preSpinLauncher)
                .build();

        farPark = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(54),14), new Pose(x(35),14))
                )
                .setTangentHeadingInterpolation()
                .build();
    }

    public void autoStateMachine18() {
        switch (autoState) {
            case -1:
                break;
            case 0:
                transferLoadSpeed = 1.0;
                follower.followPath(StartToScore);
                autoState = 1;
                premoveTurret(x(45),92,Math.toDegrees(a(135)));
                break;
            case 1:
                if (!follower.isBusy()) {
                    launchBalls();
                    waitTimer.reset();
                    autoState = 3;
                }
                break;
            case 2:
                if (waitTimer.seconds() > .1) {
                    autoState = 3;
                }
                break;
            case 3:
                if (launchTimer.seconds() > launchTime) {
                    resetSubsystems();
                    follower.followPath(ScoreToPGP18);
                    autoState = 4;
                }
                break;
            case 4:
                if (follower.atParametricEnd() && follower.getHeadingError() < follower.getCurrentPath().getPathEndHeadingConstraint()) {
                    follower.followPath(PGPIntake18,intakeMaxPower,true);
                    intakeBalls();
                    autoState = 5;
                }
                break;
            case 5:
                if (follower.atParametricEnd()) {
                    resetSubsystems();
                    launcherState = "rejecting";
                    intakeState = "intaking";
                    follower.followPath(PGPToScore18);
                    premoveTurret(x(48),84,Math.toDegrees(a(223)));
                    autoState = 6;
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    launchBalls();
                    autoState = 7;
                }
                break;
            case 7:
                if (launchTimer.seconds() > launchTime) {
                    resetSubsystems();
                    follower.followPath(ScoreToRamp1);
                    autoState = 8;
                }
                break;
            case 8:
                if (follower.getTranslationalError().getMagnitude() < 2) {
                    follower.followPath(IntakeRamp,rampMaxPower,true);
                    intakeBalls();
                    launcherState = "rejectFaster";
                    autoState = 9;
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    autoState = 10;
                    follower.holdPoint(new Pose(x(12.5),59.5,a(140)));
                    waitTimer.reset();
                }
                break;
            case 10:
                if (waitTimer.seconds() > gateWaitTime) {
                    resetSubsystems();
                    launcherState = "rejecting";
                    intakeState = "intaking";
                    premoveTurret(x(49),84,Math.toDegrees(a(216)));
                    follower.followPath(RampToScore1);
                    autoState = -5;
                }
                break;
            case -5:
                if (follower.atParametricEnd()) {
                    follower.followPath(RampToScore2);
                    autoState = 11;
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    launchBalls();
                    autoState = 12;
                }
                break;
            case 12:
                if (launchTimer.seconds() > launchTime) {
                    resetSubsystems();
                    follower.followPath(ScoreToRamp2);
                    autoState = 14;
                }
                break;
            case 14:
                if (follower.getTranslationalError().getMagnitude() < 2) {
                    follower.followPath(IntakeRamp,rampMaxPower,true);
                    intakeBalls();
                    launcherState = "rejectFaster";
                    telemetryM.addData("Drive error",follower.getTranslationalError().getMagnitude());
                    autoState = 15;
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    autoState = 16;
                    waitTimer.reset();
                    follower.holdPoint(new Pose(x(12.5),59.5,a(140)));
                }
                break;
            case 16:
                if (waitTimer.seconds() > gateWaitTime) {
                    resetSubsystems();
                    launcherState = "rejecting";
                    intakeState = "intaking";
                    premoveTurret(x(49),84,Math.toDegrees(a(216)));
                    follower.followPath(RampToScore1);
                    autoState = -6;
                }
                break;
            case -6:
                if (follower.getDriveError() < 2) {
                    follower.followPath(RampToScore2);
                    autoState = 17;
                }
                break;
            case 17:
                if (!follower.isBusy()) {
                    launchBalls();
                    autoState = 19;
                }
                break;
            case 18:
                if (launchTimer.seconds() > launchTime) {
                    resetSubsystems();
                    //follower.turnTo(a(180));
                    autoState = 19;
                }
                break;
            case 19:
                if (launchTimer.seconds() > launchTime) {
                    resetSubsystems();
                    follower.followPath(PPGIntake18,intakeMaxPower,true);
                    intakeBalls();
                    autoState = 20;
                }
                break;
            case 20:
                if (follower.atParametricEnd()) {
                    resetSubsystems();
                    launcherState = "rejecting";
                    intakeState = "intaking";
                    premoveTurret(x(49),84,Math.toDegrees(a(180)));
                    follower.followPath(PPGToScore);
                    autoState = 21;
                }
                break;
            case 21:
                if (!follower.isBusy()) {
                    launchBalls();
                    autoState = 22;
                }
                break;
            case 22:
                if (launchTimer.seconds() > launchTime) {
                    resetSubsystems();
                    follower.followPath(ScoreToGPP);
                    autoState = 24;
                }
                break;
            case 23:
                if (!follower.isBusy()) {
                    follower.turnTo(a(180));
                    autoState = 24;
                }
                break;
            case 24:
                if (!follower.isBusy()) {
                    follower.followPath(GPPIntake18,intakeMaxPower,true);
                    intakeBalls();
                    autoState = 25;
                }
                break;
            case 25:
                if (follower.atParametricEnd()) {
                    resetSubsystems();
                    launcherState = "rejecting";
                    intakeState = "intaking";
                    premoveTurret(x(56),80,Math.toDegrees(a(230)));
                    follower.followPath(GPPToScore18one);
                    autoState = -7;
                }
                break;
            case -7:
                if (follower.atParametricEnd()) {
                    follower.followPath(GPPToScore18two);
                    autoState = 26;
                }
                break;
            case 26:
                if (!follower.isBusy()) {
                    launchBalls();
                    autoState = 27;
                }
                break;
            case 27:
                if (launchTimer.seconds() > launchTime) {
                    follower.followPath(Park18);
                    resetSubsystems();
                    autoState = 28;
                }
                break;
            case 28:
                telemetryM.addLine("Autonomous Complete!!! :D");
                endPosition = follower.getPose();
                break;
        }
    }

    public void buildPaths18() { // capital first letter means for the 18 auto
        StartToScore = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(18.5), 118.5), new Pose(x(45), 92))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addTemporalCallback(500, preSpinLauncher)
                .setBrakingStrength(0.68)
                .build();

        ScoreToPGP18 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(45), 92), new Pose(x(45), 60))
                )
                .setLinearHeadingInterpolation(a(135), a(180),0.8)
                //.setBrakingStrength(0.6)
                .build();

        PGPIntake18 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(45), 60), new Pose(x(22), 60))
                )
                .setTangentHeadingInterpolation()
                //.setBrakingStrength(0.6)
                .build();

        PGPToScore18 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(22), 60), new Pose(x(48), 84))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addTemporalCallback(200,stopIntake)
                .addTemporalCallback(300, rejectIntake)
                .addTemporalCallback(400, stopIntake)
                .addTemporalCallback(500, preSpinLauncher)
                .setBrakingStrength(0.66)
                .build();

        ScoreToRamp1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(48), 84), new Pose(x(35), 63))
                )
                .setLinearHeadingInterpolation(a(223), a(140),0.8)
                .build();

        IntakeRamp = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(x(32), 59.5),
                                new Pose(x(12.5), 59.5)
                        )
                )
                .setConstantHeadingInterpolation(a(140))
                .build();

        RampToScore1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(13.0),59.5), new Pose(x(16),60))
                )
                .setConstantHeadingInterpolation(a(140))
                .setNoDeceleration()
                .addTemporalCallback(200,stopIntake)
                .addTemporalCallback(300, rejectIntake)
                .addTemporalCallback(400, stopIntake)
                .addTemporalCallback(500, preSpinLauncher)
                .build();

        RampToScore2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(17), 60), new Pose(x(49), 84))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .setBrakingStrength(0.66)
                .build();

        ScoreToRamp2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(49), 84), new Pose(x(35), 63))
                )
                .setLinearHeadingInterpolation(a(213), a(140),0.8)
                .build();

        //follower.turnTo(a(180));
        PPGIntake18 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(49), 84), new Pose(x(20), 84))
                )
                .setTangentHeadingInterpolation()
                .build();

        PPGToScore = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(20), 84), new Pose(x(49), 84))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addTemporalCallback(200,stopIntake)
                .addTemporalCallback(300, rejectIntake)
                .addTemporalCallback(400, stopIntake)
                .addTemporalCallback(500, preSpinLauncher)
                .setBrakingStrength(0.68)
                .build();

        ScoreToGPP = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(49), 84), new Pose(x(48), 65))
                )
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .addPath(
                        new BezierLine(new Pose(x(48),65), new Pose(x(48),39))
                )
                .setLinearHeadingInterpolation(a(270),a(180),.8)
                .build();

        //follower.turnTo(a(180));
        GPPIntake18 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(48), 36), new Pose(x(20), 36))
                )
                .setTangentHeadingInterpolation()
                .build();

        GPPToScore18one = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(20), 36), new Pose(x(38),58))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .setNoDeceleration()
                .addTemporalCallback(200,stopIntake)
                .addTemporalCallback(300, rejectIntake)
                .addTemporalCallback(400, stopIntake)
                .addTemporalCallback(500, preSpinLauncher)
                .build();

        GPPToScore18two = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(38),58),new Pose(x(56),80))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Park18 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(56),80), new Pose(x(48),72))
                )
                .setTangentHeadingInterpolation()
                .build();
    }

    public void autoStateMachine15() {
        switch (autoState) {
            case -1:
                follower.setMaxPower(1);
                break;
            case 0:
                transferLoadSpeed = 1.0;
                follower.followPath(startToScore15);
                autoState = 1;
                premoveTurret(x(45),92,Math.toDegrees(a(135)));
                break;
            case 1:
                if (!follower.isBusy() && follower.getVelocity().getMagnitude() < 1.0) {
                    launchBalls();
                    waitTimer.reset();
                    autoState = 3;
                }
                break;
            case 3:
                if (launchTimer.seconds() > launchTime) {
                    resetSubsystems();
                    follower.followPath(scoreToPGP15);
                    autoState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(PGPIntake15,intakeMaxPower15,true);
                    intakeBalls();
                    autoState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    resetSubsystems();
                    launcherState = "rejecting";
                    intakeState = "intaking";
                    follower.followPath(PGPToScore15);
                    premoveTurret(x(50),84,Math.toDegrees(a(219)));
                    autoState = 6;
                }
                break;
            case 6:
                if (!follower.isBusy() && follower.getVelocity().getMagnitude() < 1.0) {
                    launchBalls();
                    autoState = 7;
                }
                break;
            case 7:
                if (launchTimer.seconds() > launchTime15) {
                    resetSubsystems();
                    follower.followPath(ScoreToRamp115);
                    autoState = -8;
                }
                break;
            case -8:
                if (!follower.isBusy()) {
                    follower.followPath(ScoreToRamp215,rampMaxPower15,true);
                    autoState = -9;
                    waitTimer.reset();
                }
                break;
            case -9:
                if (waitTimer.seconds() > 0.4) {
                    autoState = 8;
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(IntakeRamp15);
                    intakeBalls();
                    launcherState = "rejectFaster";
                    autoState = 9;
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    autoState = 10;
                    follower.holdPoint(new Pose(x(13),52,a(140)));
                    waitTimer.reset();
                }

                break;
            case 10:
                if (waitTimer.seconds() > gateWait15) {
                    resetSubsystems();
                    launcherState = "rejecting";
                    intakeState = "intaking";
                    premoveTurret(x(50),84,Math.toDegrees(a(216)));
                    follower.followPath(RampToScore115);
                    autoState = -5;
                }
                break;
            case -5:
                if (follower.atParametricEnd()) {
                    follower.followPath(RampToScore215);
                    autoState = 11;
                }
                break;
            case 11:
                if (!follower.isBusy() && follower.getVelocity().getMagnitude() < 1.0) {
                    launchBalls();
                    autoState = 19;
                }
                break;
            case 19:
                if (launchTimer.seconds() > launchTime15) {
                    resetSubsystems();
                    follower.followPath(PPGIntake15,intakeMaxPower15,true);
                    intakeBalls();
                    autoState = 20;
                }
                break;
            case 20:
                if (follower.atParametricEnd()) {
                    resetSubsystems();
                    launcherState = "rejecting";
                    intakeState = "intaking";
                    premoveTurret(x(50),84,Math.toDegrees(a(180)));
                    follower.followPath(PPGToScore15);
                    autoState = 21;
                }
                break;
            case 21:
                if (!follower.isBusy() && follower.getVelocity().getMagnitude() < 1.5) {
                    launchBalls();
                    autoState = 22;
                }
                break;
            case 22:
                if (launchTimer.seconds() > launchTime15) {
                    resetSubsystems();
                    follower.followPath(ScoreToGPP15);
                    autoState = 24;
                }
                break;
            case 24:
                if (!follower.isBusy()) {
                    follower.followPath(GPPIntake15,intakeMaxPower15,true);
                    intakeBalls();
                    autoState = 25;
                }
                break;
            case 25:
                if (!follower.isBusy()) {
                    resetSubsystems();
                    launcherState = "rejecting";
                    intakeState = "intaking";
                    premoveTurret(x(60),80,Math.toDegrees(a(238)));
                    follower.followPath(GPPToScore15one);
                    autoState = 26;
                }
                break;
            case 26:
                if (!follower.isBusy() && follower.getVelocity().getMagnitude() < 1.5) {
                    launchBalls();
                    autoState = 27;

                }
                break;
            case 27:
                if (launchTimer.seconds() > launchTime15) {
                    follower.followPath(Park15);
                    resetSubsystems();
                    autoState = 28;
                }
                break;
            case 28:
                telemetryM.addLine("15 Autonomous Complete!!! :D");
                endPosition = follower.getPose();
                break;
        }
    }

    public void buildPaths15() {
        startToScore15 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(18.5), 118.5), new Pose(x(45), 92))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addTemporalCallback(500, preSpinLauncher)
                .setBrakingStrength(0.65)
                .build();

        scoreToPGP15 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(45), 92), new Pose(x(45), 60))
                )
                .setLinearHeadingInterpolation(a(135), a(180),0.8)
                //.setBrakingStrength(0.6)
                .build();

        PGPIntake15 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(45), 60), new Pose(x(19), 60))
                )
                .setTangentHeadingInterpolation()
                //.setBrakingStrength(0.6)
                .build();

         PGPToScore15 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(19), 60), new Pose(x(50), 86))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addTemporalCallback(200,stopIntake)
                .addTemporalCallback(300, rejectIntake)
                .addTemporalCallback(400, stopIntake)
                .addTemporalCallback(500, preSpinLauncher)
                .setBrakingStrength(0.66)
                .build();

        ScoreToRamp115 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(50), 86), new Pose(x(35), 63))
                )
                .setLinearHeadingInterpolation(a(223), a(135),0.8)
                .build();

        ScoreToRamp215 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(x(32), 59.5),
                                new Pose(x(12.5), 59.5)
                        )
                )
                .setConstantHeadingInterpolation(a(135))
                .build();

        IntakeRamp15 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(x(12.5), 59.5),
                                new Pose(x(12.5), 52)
                        )
                )
                .setConstantHeadingInterpolation(a(135))
                .build();

        RampToScore115 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(12.5),52), new Pose(x(25),60))
                )
                .setConstantHeadingInterpolation(a(135))
                .setNoDeceleration()
                .addTemporalCallback(200,stopIntake)
                .addTemporalCallback(300, rejectIntake)
                .addTemporalCallback(400, stopIntake)
                .addTemporalCallback(500, preSpinLauncher)
                .build();

        RampToScore215 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(25), 60), new Pose(x(52), 86))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .setBrakingStrength(0.66)
                .build();

        PPGIntake15 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(52), 86), new Pose(x(21), 84))
                )
                .setTangentHeadingInterpolation()
                .build();

        PPGToScore15 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(21), 84), new Pose(x(52), 86))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addTemporalCallback(200,stopIntake)
                .addTemporalCallback(300, rejectIntake)
                .addTemporalCallback(400, stopIntake)
                .addTemporalCallback(500, preSpinLauncher)
                .setBrakingStrength(0.68)
                .build();

        ScoreToGPP15 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(52), 86), new Pose(x(48), 65))
                )
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .addPath(
                        new BezierLine(new Pose(x(48),65), new Pose(x(48),39))
                )
                .setLinearHeadingInterpolation(a(270),a(180),.8)
                .build();

        //follower.turnTo(a(180));
        GPPIntake15 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(48), 36), new Pose(x(16), 36))
                )
                .setTangentHeadingInterpolation()
                .build();

        GPPToScore15one = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(16), 36), new Pose(x(52),86))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addTemporalCallback(200,stopIntake)
                .addTemporalCallback(300, rejectIntake)
                .addTemporalCallback(400, stopIntake)
                .addTemporalCallback(500, preSpinLauncher)
                .build();


        Park15 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(50),84), new Pose(x(48),72))
                )
                .setTangentHeadingInterpolation()
                .build();
    }

    private void autoStateMachine12() {
        switch (autoState) {
            case -1:
                break;
            case 0:
                transferLoadSpeed = 1.0;
                follower.followPath(startToScore);
                autoState = 1;
                premoveTurret(x(45),92,Math.toDegrees(a(135)));
                break;
            case 1:
                if (!follower.isBusy()) {
                    launchTimer.reset();
                    autoState = -6;
                }
                break;
            case -6:
                if (launchTimer.seconds() > .3) {
                    launcherState = "beginLaunchSequence";
                    autoState = 3;
                }
                break;
            case 3:
                if (launchTimer.seconds() > launchTime) {
                    resetSubsystems();
                    follower.followPath(scoreToPPG);
                    autoState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(PPGIntake,intakeMaxPower,true);
                    intakeBalls();
                    autoState = -2;
                }
                break;
            case -2:
                if (!follower.isBusy()) {
                    follower.turnTo(a(270));
                    autoState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(PPGToRamp);
                    resetSubsystems();
                    autoState = -8;
                }
                break;
            case -8:
                if (!follower.isBusy()) {
                    launchTimer.reset();
                    autoState = 6;
                }
                break;
            case 6:
                if (launchTimer.seconds()> 0.75) {
                    follower.followPath(rampToScore);
                    premoveTurret(x(45),101,Math.toDegrees(a(225)));
                    autoState = 7;
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    launcherState = "beginLaunchSequence";
                    autoState = 8;
                }
                break;
            case 8:
                if (launchTimer.seconds() > launchTime) {
                    resetSubsystems();
                    //follower.turnTo(a(270));
                    autoState = 9;
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(scoreToPGP);
                    autoState = 10;
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.turnTo(a(180));
                    autoState = 11;
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    intakeBalls();
                    follower.followPath(PGPIntake,intakeMaxPower,true);
                    autoState = 12;
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    resetSubsystems();
                    premoveTurret(x(50),94,Math.toDegrees(a(225)));
                    //follower.turnTo(a(225));
                    autoState = 13;
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(PGPToScore);
                    autoState = 14;
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    launcherState = "beginLaunchSequence";
                    autoState = 15;
                }
                break;
            case 15:
                if (launchTimer.seconds() > launchTime) {
                    resetSubsystems();
                    //follower.turnTo(a(270));
                    autoState = 16;
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    follower.followPath(scoreToGPP);
                    autoState = 17;
                }
                break;
            case 17:
                if (!follower.isBusy()) {
                    follower.turnTo(a(180));
                    autoState=18;
                }
                break;
            case 18:
                if (!follower.isBusy()) {
                    intakeBalls();
                    follower.followPath(GPPIntake,intakeMaxPower,true);
                    autoState = 19;
                }
                break;
            case 19:
                if (!follower.isBusy()) {
                    resetSubsystems();
                    //follower.turnTo(a(232.4));
                    autoState=-4;
                }
                break;
            case -4:
                if (!follower.isBusy()) {
                    premoveTurret(x(56),88,Math.toDegrees(a(232.3)));
                    follower.followPath(GPPToScore);
                    autoState = 20;
                }
            case 20:
                if (!follower.isBusy()) {
                    launcherState = "beginLaunchSequence";
                    autoState = 21;
                }
                break;
            case 21:
                if (launchTimer.seconds() > launchTime) {
                    resetSubsystems();
                    follower.followPath(Park);
                    autoState = 22;
                }
                break;
            case 22:
                break;
        }
    }

    public void buildPaths12() {

        startToScore = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(18.5), 118.5), new Pose(x(45), 92))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addParametricCallback(.35, preSpinLauncher)
                .build();
        
        scoreToPPG = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(45), 92), new Pose(x(50), 84))
                )
                .setLinearHeadingInterpolation(a(135), a(180),0.8)
                .build();

        PPGIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(50), 84), new Pose(x(23), 84))
                )
                .setTangentHeadingInterpolation()
                .build();

        PPGToRamp = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(x(23), 84),
                                new Pose(x(23), 73),
                                new Pose(x(16.5), 72)
                        )
                )
                .setConstantHeadingInterpolation(a(270))
                //.setLinearHeadingInterpolation(a(180), a(270))
                .build();

        rampToScore = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(16.5),72), new Pose(20,72))
                )
                .setConstantHeadingInterpolation(a(270))
                .addPath(
                        new BezierLine(new Pose(x(20), 72), new Pose(x(45), 101))
                )
                .setTangentHeadingInterpolation()
                //.setLinearHeadingInterpolation(a(90), a(45))
                .addParametricCallback(.35, preSpinLauncher)
                .setReversed()
                .build();

        scoreToPGP = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(45), 101), new Pose(x(49), 60))
                )
                .setTangentHeadingInterpolation()
                .build();

        //follower.turnTo(a(180));
        PGPIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(49), 60), new Pose(x(22), 60))
                )
                .setTangentHeadingInterpolation()
                .build();

        //follower.turnTo(a(225));
        PGPToScore = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(22), 60), new Pose(x(50), 94))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addParametricCallback(.35, preSpinLauncher)
                .build();

        //follower.turnTo(a(85));
        scoreToGPP = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(50), 94), new Pose(x(49), 36))
                )
                .setTangentHeadingInterpolation()
                .build();

        //follower.turnTo(a(180));
        GPPIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(49), 36), new Pose(x(22), 36))
                )
                .setTangentHeadingInterpolation()
                .build();

        //follower.turnTo(a(52.4));
        GPPToScore = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(22), 36), new Pose(x(52), 92))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addParametricCallback(.35, preSpinLauncher)
                .build();

        Park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(52), 92), new Pose(x(48), 78))
                )
                .setTangentHeadingInterpolation()
                .build();

    }
}

