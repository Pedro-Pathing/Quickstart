package org.firstinspires.ftc.teamcode.premier;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.utils.LoopTimer;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

public class PremierAuto extends LinearOpMode {

    //use public static values for team, endpose, maybe position?
    @IgnoreConfigurable
    TelemetryManager telemetryM;

    private double lastLaunchSpeed = 0;

    public static double launchTime = 1.0;
    public static double loadTime = 0.6;
    public static double gateTime = 1.5;
    int fullCount = 0;

    private double headingEstimate;

    private final ElapsedTime intakeTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private final ElapsedTime loadingTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private final ElapsedTime gateTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private final ElapsedTime launchTimeout = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    public static double hoodRapidMultiplier = 0.015;
    public static double baseLoadSpeed = .6;
    public static double loadSpeedValue = 20; //Max range = 150, min = 40. as example, 10/40=base+.25
    public static double bestAngle = 48; //projectile best angle for hood compensation
    public int loopCount = 0;
    private LoopTimer time;
    private boolean ballIn1=false, ballIn2=false, ballIn3=false;

    private double idleSpeed;

    public Pose currentPose;
    public Pose scorePose = new Pose(58,79);
    public Pose startPose = new Pose(19.5,121.000,Math.toRadians(144));
    public Pose goalPose = new Pose(PremierTeleOp.goalX, PremierTeleOp.goalY);
    double goalX, goalY;

    public static enum teamEnum {red, blue}
    public static teamEnum currentTeam = teamEnum.blue;
    public enum AUTO {closeStart, closePGP, closePPG, closeGate,done}
    public AUTO currentStep;
    public int autoState = 0;
    public AUTO[] close15 = {AUTO.closeStart,AUTO.closePGP,AUTO.closeGate,AUTO.closeGate,AUTO.closePPG, AUTO.done};
    public AUTO[] selectedAuto = close15;
    private boolean stepIsComplete = false;
    public int index = 0;
    public boolean launchCompleted = false;
    public boolean isIntakeFull = false;

    //Pathchain

    public PathChain startToScoreC, scoreToPGPC, scoreToPGPCIntake, gateClosePGPC, gatePGPToScoreC, scoreToGateC, gateToScoreC, scoreToPPGC, scoreToPPGCIntake, PPGToScoreC;

    //Devices
    private MotorEx fL, fR, bL, bR, l1, l2, mTurret, mIntake;
    private MotorGroup launcher;
    private ServoEx sBlocker, sHood;
    private DistanceSensor dist1, dist2, dist3;
    private ColorSensor color1;
    private MecanumDrive drive;
    private AnalogInput absEncoder;

    //Bot Info
    private double voltage;
    VoltageSensor voltageSensor;

    //Launcher Variables:
    public static double lkP = PremierTeleOp.lkP;
    public static double lkI = PremierTeleOp.lkI;
    public static double lkD = PremierTeleOp.lkD;
    public static double lkV = PremierTeleOp.lkV;
    public static double lkS = PremierTeleOp.lkS;
    private double launchRange = 0;
    private double launcherTargetSpeed;
    private customPIDF lController, tController;
    double launcherInput=0;
    public enum lState {off, idle, waiting, firing}
    public enum bState {open, closed, disabled};
    public bState blockerState = bState.open;
    public lState launcherState = lState.off;
    public InterpLUT velLUT = new InterpLUT();
    public InterpLUT angleLUT = new InterpLUT();
    public double minRange=40;
    public double maxRange=165;
    public boolean isFiring = false;
    public double launcherTargetSpeedTemp = 0;

    //HOod Variables
    public static double closedAngle = PremierTeleOp.closedAngle;
    public static double openAngle = PremierTeleOp.openAngle;
    public double hoodTargetAngle = 40;
    public static double minHoodAngle = PremierTeleOp.minHoodAngle;
    public static double maxHoodAngle = PremierTeleOp.maxHoodAngle;
    public static double hoodOffset = PremierTeleOp.hoodOffset;
    public static double testHoodAngle = PremierTeleOp.testHoodAngle;

    //Turret Variables:
    public static double tKP = PremierTeleOp.tKP;
    public static double tKI = PremierTeleOp.tKI;
    public static double tKD = PremierTeleOp.tKD;
    public static double tKV = PremierTeleOp.tKV;
    public static double tKS = PremierTeleOp.tKS;
    public static double turretTolerance = PremierTeleOp.turretTolerance;
    public enum tState {off, tracking, aiming, aimed, premove}
    public tState turretState = tState.off;
    private double turretTargetAngle, angleError, turretInput;
    public boolean turretAngleLimited = false;
    public boolean manualControl = false;

    //Absolute Encoder Variables:
    public static double absOffset = PremierTeleOp.absOffset;
    double outputVoltage=0;
    double outputAngle=0;
    double totalRawAngle = 0;
    double turnCounter = 0;
    double turretAngle = 0;
    double previousAngle = 0;
    double previousRawAngle = 0;

    //Intake Variables:
    private double alpha1, distance2, distance3;
    private double intakeSpeed = 1.0;
    private double rejectSpeed = -.4;
    public enum iState {idle, intaking, loading, rejecting, stuckBall, gate}
    public iState intakeState = iState.idle;



    public void initializeAuto() {
        scorePose = new Pose(x(scorePose.getX()),scorePose.getY());
        startPose = new Pose(x(startPose.getX()),startPose.getY(),a(startPose.getHeading()));
        goalPose = new Pose(x(goalPose.getX()),goalPose.getY());
        goalX = goalPose.getX();
        goalY = goalPose.getY();
        currentStep = selectedAuto[0];

        buildPaths();
        idleSpeed = velLUTClipped(Math.hypot(goalPose.getX()- scorePose.getX(), goalPose.getY())-scorePose.getY());
        turretState = tState.tracking;
    }

    public void inInit() {
        if (gamepad1.left_bumper) {
            currentTeam = teamEnum.blue;
        }
        if (gamepad1.right_bumper) {
            currentTeam = teamEnum.red;
        }
        telemetryM.addLine("Left Stick = blue");
        telemetryM.addLine("Right Stick = red");
        telemetryM.addData("Team:",currentTeam);
        telemetryM.addLine("Auto selection");
        telemetryM.addData("Selected Auto:",selectedAuto);
        telemetryM.update(telemetry);
    }

    @Override
    public void runOpMode() {
        follower= Constants.createFollower(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        initHardware();
        createLUTS();
        //Set up all instant stuff, then auto selection loop

        while (opModeInInit()) {
            inInit();
        }

        //Last thing before opmode stop
        initializeAuto();


        while (opModeIsActive()) {
            follower.update();
            currentPose=follower.getPose();
            time.start();
            hubs.forEach(LynxModule::clearBulkCache);
            runOpModeStep();
            auto();
            time.end();
            telemetry();
        }
        //have a 30sec timer, maybe push for 30.5sec but instead of real timeout continue tracking position for accurate endpose?
    }

    public void auto() {
        sensorUpdate(false);
        turretUpdate();
        launcherUpdate();
        intakeUpdate();
    }

    public void sensorUpdate(boolean atGate) {
        loopCount += 1;
        loopCount = loopCount % 9;
        if (!atGate) {
            switch (loopCount) {
                case 0:
                    distance2 = dist2.getDistance(DistanceUnit.INCH);
                    ballIn2 = distance2 < 3;
                    break;
                case 3:
                    distance3 = dist3.getDistance(DistanceUnit.INCH);
                    ballIn3 = distance3 < 4;
                    break;
                case 6:
                    alpha1 = color1.alpha();
                    ballIn1 = alpha1 > 80;
                    break;
            }
        }
        else {
            distance2 = dist2.getDistance(DistanceUnit.INCH);
            ballIn2 = distance2 < 3;
            distance3 = dist3.getDistance(DistanceUnit.INCH);
            ballIn3 = distance3 < 4;
            alpha1 = color1.alpha();
            ballIn1 = alpha1 > 90;
        }

    }

    public void turretUpdate() {
        //Absolute Encoder
        {
            previousAngle = outputAngle;
            previousRawAngle = totalRawAngle;
            outputVoltage = absEncoder.getVoltage();
            outputAngle = outputVoltage / 5 * 360;
            if (Math.abs(outputAngle-previousAngle) > 180) {
                if (outputAngle < previousAngle) turnCounter += 1;
                else if (outputAngle > previousAngle) turnCounter -= 1;
            }
            totalRawAngle = outputAngle + turnCounter*360;
            turretAngle = ((totalRawAngle + absOffset)/2);

        }
        //Turret Position Logic
        switch (turretState) {
            case off:

                break;
            case tracking:
                turretTargetAngle = calculateTurretAngle(currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
                turretTargetAngle = limitAngle(turretTargetAngle);
                break;
            case aiming:
                turretTargetAngle = calculateTurretAngle(currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
                turretTargetAngle = limitAngle(turretTargetAngle);
                if (Math.abs(tController.getPositionError()) < turretTolerance) turretState = tState.aiming;
                break;
            case aimed:
                turretTargetAngle = calculateTurretAngle(currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
                turretTargetAngle = limitAngle(turretTargetAngle);
                if (Math.abs(tController.getPositionError()) > turretTolerance) turretState = tState.aimed;
                break;
            case premove:
                turretTargetAngle = calculateTurretAngle(scorePose.getX(), scorePose.getY(), Math.toDegrees(headingEstimate));
                turretTargetAngle = limitAngle(turretTargetAngle);
                break;
        }
        if (turretState != tState.off){
            turretTargetAngle = calculateTurretAngle(currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
            turretTargetAngle = limitAngle(turretTargetAngle);
        }
        tController.setPIDF(tKP,tKI,tKD,tKV,tKS,voltage);
        if (!manualControl) turretInput = tController.calculate(turretAngle, turretTargetAngle);
        if (Math.abs(tController.getPositionError()) < turretTolerance && turretState == tState.aiming) turretState = tState.aimed;
        else if (turretState == tState.aimed && Math.abs(tController.getPositionError()) > turretTolerance) turretState = tState.aiming;
        mTurret.set(turretInput);
    }

    public double calculateTurretAngle(double botX, double botY, double botHeading) {
        double goalX = goalPose.getX();
        double goalY = goalPose.getY();
        launchRange = Math.hypot(goalX-botX,goalY-botY);
        if (botY > 130) {
            goalY -= 5;
        } else if (botY < 40) {
            goalX = x(5);
        }
        double targetAngle = Math.toDegrees(Math.atan2(goalY-botY,goalX-botX));
        targetAngle -= botHeading + 180;
        return targetAngle;
    }

    public double limitAngle(double angle) {
        turretAngleLimited = false;
        double angleGoal = angle;
        if (angleGoal > 180) {
            angleGoal -= 360;
        } else if (angleGoal < -180) {
            angleGoal += 360;
        }
        if ((angleGoal > 160 && turretAngle < 0) || (angleGoal < -160 && turretAngle > 0)) {
            turretAngleLimited = true;
            return turretAngle;
        }
        if (angleGoal > 150) {
            angleGoal = 150;
            turretAngleLimited = true;
        } else if (angleGoal < -150) {
            angleGoal = -150;
            turretAngleLimited = true;
        }
        return angleGoal;
    }

    public void launcherUpdate() {
        goalPose = new Pose(goalX,goalY);
        //Launch logic outline:
        //FSM - Idle state, waiting for turret state, active launch state with checks for extreme movement, sensor check to finish
        //Make a driver 2 toggle between close and far, assuming usually close play,
        //so prep speed reasonable if needing to go far for balls
        switch (launcherState) {
            case off:
                launcherTargetSpeed = 0;
                break;
            case idle:
                //may need to make it only spin up if too low to maintain, not decrease suddenly, preventing undue power use
                //assuming launch position is roughly the same -- maybe even making it copy previous launch position?
                //maybe have somehting to begin adjusting once it comes back into launch zone - checking 4 corners & edges w/ trig
                launcherTargetSpeed = idleSpeed;
                isFiring = false;

                //test idle mode: averages idle speed with last launch speed
                break;
            case firing:
                if (!isFiring) {
                    isFiring = true;
                    loadingTimer.reset();
                }
                if (loadingTimer.seconds() > loadTime) {
                    launchCompleted = true;
                }
                intakeState = iState.loading;
                launcherTargetSpeedTemp = velLUTClipped(launchRange);
                if (launcher.getVelocity() > launcherTargetSpeedTemp + 20) launcherTargetSpeed=0;
                else launcherTargetSpeed=5000; //sufficiently high to get max power response
                if (turretState == tState.aiming) launcherState = lState.waiting;
                break;
            case waiting:
                isFiring = false;
                launcherTargetSpeed = velLUTClipped(launchRange);
                if (intakeState == iState.loading) intakeState = iState.idle;
                if (turretState == tState.aimed && follower.getVelocity().getMagnitude() < 3 && (Math.abs(launcherTargetSpeed-launcher.getVelocity())) < 40) launcherState = lState.firing;
                break;
        }
        lController.setPIDF(lkP,lkI,lkD,lkV,lkS,voltage);
        if (launcherTargetSpeed != 0) launcherInput = lController.calculate(launcher.getVelocity(),launcherTargetSpeed);
        else launcherInput = 0;
        launcher.set(launcherInput);
        hoodTeleOp();
        switch (blockerState) {
            case open:
                sBlocker.set(openAngle);
                break;
            case closed:
                sBlocker.set(closedAngle);
                break;
            case disabled:
                sBlocker.disable();
        }
    }

    public void hoodTeleOp() {
        if (launchRange <= minRange) hoodTargetAngle = minHoodAngle;
        else if (launchRange >= maxRange) hoodTargetAngle = angleLUT.get(maxRange-0.1);
        else hoodTargetAngle = angleLUT.get(launchRange);

        if (hoodTargetAngle<bestAngle && launchRange > 50) {
            hoodTargetAngle += ((launcherTargetSpeedTemp - launcher.getVelocity()) * hoodRapidMultiplier);
            if (hoodTargetAngle > bestAngle) hoodTargetAngle = bestAngle;
        }
        else if (hoodTargetAngle>bestAngle) {
            hoodTargetAngle -= ((launcherTargetSpeedTemp - launcher.getVelocity()) * hoodRapidMultiplier);
            if (hoodTargetAngle < bestAngle) hoodTargetAngle = bestAngle;
        }
        if (hoodTargetAngle < minHoodAngle) hoodTargetAngle = minHoodAngle;
        if (hoodTargetAngle > maxHoodAngle) hoodTargetAngle = maxHoodAngle;
        //Manual for testing:
        // hoodTargetAngle = testHoodAngle;

        double hoodTargetPosition = (double) 177/24 * (hoodTargetAngle-minHoodAngle-hoodOffset); //?????
        sHood.set(hoodTargetPosition);
    }

    public void intakeUpdate() {
        switch (intakeState){
            case idle:
                mIntake.set(0);
                break;
            case intaking:
//                if (!ballIn1 || !ballIn2 || !ballIn3) {
//                    mIntake.set(intakeSpeed);
//                }
//                else {
//                    mIntake.set(0);
//                }
                mIntake.set(intakeSpeed);
                //move blocker into position - find a way to prevent jam if ball already in? Maybe only move blocker when full
                //or trying to launch?
                break;
            case loading:
                double realLoadSpeed = baseLoadSpeed + loadSpeedValue /launchRange;
                mIntake.set(realLoadSpeed);
                //definitely make sure blocker out of the way
                break;
            case rejecting:
                mIntake.set(rejectSpeed);
                //maybe blocker out of the way in case of weird jam??
                break;
            case stuckBall:
                if (intakeTimer.seconds() < 0.10) mIntake.set(-1);
                else mIntake.set(1);
                break;
            case gate:
                sensorUpdate(true);
                if (ballIn1 && ballIn2 && ballIn3) fullCount += 1;
                if (fullCount >= 4) {
                    isIntakeFull = true;
                } else {
                    isIntakeFull = false;
                }
                mIntake.set(intakeSpeed);
                break;
        }
    }

    public void telemetry() {
        voltage = voltageSensor.getVoltage();
        telemetryM.addData("Launcher Vel",launcher.getVelocity());
        telemetryM.addData("Launcher Target",launcherTargetSpeed);
        telemetryM.addData("Launcher Power",launcher.get());
        telemetryM.addData("Voltage",voltage);
        telemetryM.addData("Target",turretTargetAngle);
        telemetryM.addData("Turret Angle", turretAngle);
        telemetryM.addData("input turret",turretInput);
        telemetryM.addData("Raw angle",totalRawAngle);
        telemetryM.addData("Abs Angle",outputAngle);
        telemetryM.addData("X",currentPose.getX());
        telemetryM.addData("Y",currentPose.getY());
        telemetryM.addData("Heading",Math.toDegrees(currentPose.getHeading()));
        telemetryM.addData("Range",launchRange);
        telemetryM.addData("Looptime (hz)", time.getHz());
//        telemetryM.addData("Alpha (sensor 1)",alpha1);
//        telemetryM.addData("Dist (2)",distance2);
//        telemetryM.addData("Dist (3)",distance3);
//        telemetryM.addData("Ballin1",ballIn1);
//        telemetryM.addData("Ballin2",ballIn2);
//        telemetryM.addData("Ballin3",ballIn3);
        telemetryM.update(telemetry);
    }

    public void initHardware() {
        fL = new MotorEx(hardwareMap, "fL", Motor.GoBILDA.RPM_312);
        fR = new MotorEx(hardwareMap, "fR", Motor.GoBILDA.RPM_312);
        bL = new MotorEx(hardwareMap, "bL", Motor.GoBILDA.RPM_312);
        bR = new MotorEx(hardwareMap, "bR", Motor.GoBILDA.RPM_312);
        l1 = new MotorEx(hardwareMap, "launcherMotor1", Motor.GoBILDA.BARE);
        l2 = new MotorEx(hardwareMap, "launcherMotor2", Motor.GoBILDA.BARE);
        mIntake = new MotorEx(hardwareMap,"intakeMotor",Motor.GoBILDA.BARE);
        mTurret = new MotorEx(hardwareMap,"turretMotor",Motor.GoBILDA.RPM_435);
        mTurret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        l1.setInverted(true);
        launcher = new MotorGroup(l1, l2);

        color1 = hardwareMap.get(ColorSensor.class,"colorSensor");
        dist2 = hardwareMap.get(DistanceSensor.class, "distanceSensor2");
        dist3 = hardwareMap.get(DistanceSensor.class, "distanceSensor3");

        sHood = new ServoEx(hardwareMap,"hoodServo",0, 300);
        sHood.setPwm(new PwmControl.PwmRange(500,2500));
        sHood.setInverted(true);

        sBlocker = new ServoEx(hardwareMap,"blockerServo",0,300);
        sBlocker.setPwm(new PwmControl.PwmRange(500,2500));

        absEncoder = hardwareMap.get(AnalogInput.class,"AbsoluteEncoder");

        tController = new customPIDF(tKP,tKI,tKD,tKV,tKS,0,0);
        tController.setTolerance(turretTolerance*0.5);

        lController = new customPIDF(lkP,lkI,lkD,lkV,lkS,0,0);

        drive = new MecanumDrive(fL, fR, bL, bR);
        drive.setRightSideInverted(false);

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        voltage = voltageSensor.getVoltage();
    }

    public void createLUTS() {
        velLUT.add(minRange,950);
        velLUT.add(45,990);
        velLUT.add(55,1030);
        velLUT.add(65,1050);
        velLUT.add(75,1110);
        velLUT.add(85,1170);
        velLUT.add(95,1320);
        velLUT.add(105,1360);
        velLUT.add(115,1400);
        velLUT.add(125,1440);
        velLUT.add(135,1480);
        velLUT.add(145,1520);
        velLUT.add(155,1560);
        velLUT.add(maxRange,1580);
        velLUT.createLUT();

        angleLUT.add(minRange,30);
        angleLUT.add(45,30);
        angleLUT.add(55,32);
        angleLUT.add(65,34);
        angleLUT.add(75,36);
        angleLUT.add(85,37);
        angleLUT.add(95,48);
        angleLUT.add(105,50);
        angleLUT.add(115,50);
        angleLUT.add(125,50);
        angleLUT.add(135,50);
        angleLUT.add(145,50);
        angleLUT.add(155,50);
        angleLUT.add(maxRange,50);
        angleLUT.createLUT();
    }

    public double velLUTClipped(double input) {
        if (input < minRange) return velLUT.get(minRange+0.1);
        else if (input > maxRange) return velLUT.get(maxRange-0.1);
        else return velLUT.get(input);
    }

    public void runOpModeStep() {
        switch (currentStep) {
            case closePGP:
                closePGP();
                break;
            case closePPG:
                closePPG();
                break;
            case closeGate:
                closeGate();
                break;
            case closeStart:
                closeStart();
                break;
            case done:
                //any stuff to reset, track position, etc.
                break;
        }
        if (stepIsComplete) {
            autoState = 1;
            index += 1;
            currentStep = selectedAuto[index];
            stepIsComplete = false;
        }
    }

    public void activateIntake() {
        intakeState = iState.intaking;
        blockerState = bState.closed;
    }

    public void disableIntake() {
        intakeState = iState.idle;
        blockerState = bState.open;
        isIntakeFull = false;
        //might have to add a timer or make runnable to queue in middle of path
    }

    public void launchBalls() {
        launchCompleted = false;
        launchTimeout.reset();
        launcherState = lState.waiting;
        blockerState = bState.open;
    }

    public void disableLauncher() {
        launcherState = lState.idle;
        intakeState = iState.idle;
    }

    Runnable preSpinLauncher = new Runnable() {
        @Override
        public void run() {
            turretState = tState.premove;
            blockerState = bState.open;
        }
    };
    //at the very least, opens blocker. Maybe, also turret

    Runnable gateIntakeActivate = new Runnable() {
        @Override
        public void run() {
            intakeState = iState.gate;
            blockerState = bState.open;
        }
    };


    public void closeStart() {
        switch (autoState) {
            case 1:
                //anything needed to start up stuff like laucher spinup?
                follower.followPath(startToScoreC);
                headingEstimate = startToScoreC.endPose().getHeading();
                disableIntake();
                autoState += 1;
                break;
            case 2:
                if (!follower.isBusy()) {
                    launchBalls();
                    autoState += 1;
                }
                break;
            case 3:
                if (launchCompleted || launchTimeout.seconds() > launchTime) {
                    stepIsComplete = true;
                    disableLauncher();
                }
                break;
        }
    }

    public void closeGate() {
        switch (autoState) {
            case 1:
                follower.followPath(scoreToGateC);
                autoState += 1;
                break;
            case 2:
                if (!follower.isBusy()) {
                    autoState += 1;
                    gateTimer.reset();
                    //wait for balls
                }
                break;
            case 3:
                if (isIntakeFull|| gateTimer.seconds() > gateTime) {
                    autoState += 1;
                    disableIntake();
                    follower.followPath(gateToScoreC);
                    headingEstimate = gateToScoreC.endPose().getHeading();
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    launchBalls();
                    autoState += 1;
                }
                break;
            case 5:
                if (launchCompleted || launchTimeout.seconds() > launchTime) {
                    stepIsComplete = true;
                    disableLauncher();
                }
                break;
        }

    }

    public void closePGP() {
        switch (autoState) {
            case 1:
                follower.followPath(scoreToPGPC);
                autoState += 1;
                break;
            case 2:
                if (!follower.isBusy()) {
                    autoState += 1;
                    activateIntake();
                    follower.followPath(scoreToPGPCIntake);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    autoState += 1;
                    follower.followPath(gateClosePGPC);
                    disableIntake(); //short delay to keep balls in?
                }
                break;
            case 4:
                if (!follower.isBusy()) { //may need to wait
                    autoState += 1;
                    follower.followPath(gatePGPToScoreC);
                    headingEstimate = gatePGPToScoreC.endPose().getHeading();
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    autoState += 1;
                    launchBalls();
                }
                break;
            case 6:
                if (launchCompleted || launchTimeout.seconds() > launchTime) {
                    stepIsComplete = true;
                    disableLauncher();
                }
                break;
        }

    }

    public void closePPG() {
        switch (autoState) {
            case 1:
                follower.followPath(scoreToPPGC);
                autoState += 1;
                break;
            case 2:
                if (!follower.isBusy()) {
                    autoState += 1;
                    follower.followPath(scoreToPPGCIntake);
                    activateIntake();
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    autoState += 1;
                    follower.followPath(PPGToScoreC);
                    headingEstimate = PPGToScoreC.endPose().getHeading();
                    disableIntake();
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    autoState += 1;
                    launchBalls();
                }
                break;
            case 5:
                if (launchCompleted || launchTimeout.seconds() > launchTime) {
                    stepIsComplete = true;
                    disableLauncher();
                }
                break;
        }

    }

    public double x(double input) {
        if (currentTeam == PremierAuto.teamEnum.blue) return input;
        else return 144-input;
    }

    public double a(double input) {
        if (currentTeam == PremierAuto.teamEnum.blue) return Math.toRadians(input);
        else return Math.toRadians(-(input-90)+90);
    }

    public void buildPaths() {
        startToScoreC = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(19.5), 121.000),
                                scorePose
                        )
                ).setLinearHeadingInterpolation(a(144), a(135),0.2)
                .addTemporalCallback(300, preSpinLauncher)
                .setGlobalDeceleration()
                .build();

        scoreToPGPC = follower.pathBuilder().addPath(
                        new BezierLine(
                                scorePose,
                                new Pose(x(46.0), 61.000)
                        )
                ).setLinearHeadingInterpolation(a(135), a(180))
                .setGlobalDeceleration()
                .build();

        scoreToPGPCIntake = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(46.0), 61.000),
                                new Pose(x(22.0), 61.000)
                        )
                ).setTangentHeadingInterpolation()
                .setGlobalDeceleration()
                .build();

        gateClosePGPC = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(22.0), 61.000),
                                new Pose(x(18.0), 64.000)
                        )
                ).setConstantHeadingInterpolation(a(180))
                .setGlobalDeceleration()
                .build();

        gatePGPToScoreC = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(18.0), 64.000),
                                scorePose
                        )
                ).setConstantHeadingInterpolation(a(180))
                .addTemporalCallback(300, preSpinLauncher)
                .setGlobalDeceleration()
                .build();

        scoreToGateC = follower.pathBuilder().addPath(
                        new BezierLine(
                                scorePose,
                                new Pose(x(30), 66.56)
                        )
                ).setConstantHeadingInterpolation(a(180))
                .setNoDeceleration()
                .addPath(
                        new BezierLine(
                                new Pose(x(30), 66.56),
                                new Pose(x(12.25), 59.7)
                        )
                ).setLinearHeadingInterpolation(a(180), a(146))
                .addParametricCallback(.8,gateIntakeActivate)
                .setGlobalDeceleration()
                .build();

        gateToScoreC = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(13.0), 59.000),
                                new Pose(x(18.0), 64.000)
                        )
                ).setLinearHeadingInterpolation(a(146), a(180))
                .addPath(
                        new BezierLine(
                                new Pose(x(18.0), 64.000),
                                scorePose
                        )
                ).setConstantHeadingInterpolation(a(180))
                .addTemporalCallback(300, preSpinLauncher)
                .setGlobalDeceleration()
                .build();

        scoreToPPGC = follower.pathBuilder().addPath(
                        new BezierLine(
                                scorePose,
                                new Pose(x(44.0), 84.000)
                        )
                ).setConstantHeadingInterpolation(a(180))
                .setGlobalDeceleration()
                .build();

        scoreToPPGCIntake = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(44.0), 84.000),
                                new Pose(x(22.0), 84.000)
                        )
                ).setTangentHeadingInterpolation()
                .setGlobalDeceleration()
                .build();

        PPGToScoreC = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(22.0), 84.000),
                                scorePose
                        )
                ).setConstantHeadingInterpolation(a(180))
                .setReversed()
                .addTemporalCallback(300, preSpinLauncher)
                .setGlobalDeceleration()
                .build();
    }

}
