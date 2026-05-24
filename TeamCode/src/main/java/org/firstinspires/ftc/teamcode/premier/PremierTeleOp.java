package org.firstinspires.ftc.teamcode.premier;


import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.utils.LoopTimer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@Configurable
@TeleOp(name = "TeleOp")
public class PremierTeleOp extends LinearOpMode {

    private double lastLaunchSpeed = 0;


    //HOpefully temporary
    private final ElapsedTime intakeTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private final ElapsedTime blockerTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    boolean rejectToggle = false;
    boolean blockerToggle = false;
    public static double hoodRapidMultiplier = 0.02;
    public static double baseLoadSpeed = .75;
    public static double loadSpeedValue = 15; //Max range = 150, min = 40. as example, 10/40=base+.25
    public static double bestAngle = 42; //projectile best angle for hood compensation
    public int loopCount = 0;
    private LoopTimer time;
    private boolean ballIn1=false, ballIn2=false, ballIn3=false;
    public static double goalX = 5;
    public static double goalY = 144;


    //Test Variables
    public static double testLaunchSpeed = 1200;
    public boolean turretDisabled = true;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    //Devices
    private MotorEx fL, fR, bL, bR, l1, l2, mTurret, mIntake;
    private MotorGroup launcher;
    private ServoEx sBlocker, sHood;
    private DistanceSensor dist1, dist2, dist3;
   // private ColorSensor color1;
    private MecanumDrive drive;
    private AnalogInput absEncoder;

    //Pedropathing
    private Pose currentPose;
    public static Pose startPose = new Pose (0,0,0); //has to be replaced eventually

    //Team-Dependent
    public static PremierAuto.teamEnum currentTeam = PremierAuto.currentTeam;
    public Pose goalPose = new Pose(goalX,goalY);
    public boolean isFarZone = false;


    //Drivetrain
    private double driveInput, strafeInput, turnInput;

    //Bot Info
    private double voltage;
    VoltageSensor voltageSensor;

    //Launcher Variables:
    public static double lkP = 0.00075;
    public static double lkI = 0;
    public static double lkD = 0.00001;
    public static double lkV = 0.000033;
    public static double lkS = 0;
    public static double farIdleSpeed = 1500;
    public static double closeIdleSpeed = 0; //1060 when not testing
    private double launchRange = 0;
    private double launcherTargetSpeed;
    private customPIDF lController, tController;
    double launcherInput=0;
    public enum lState {off, idle, waiting, firing, testing}
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
    public static double closedAngle = 92;
    public static double openAngle = 160;
    public double hoodTargetAngle = 40;
    public static double minHoodAngle = 30;
    public static double maxHoodAngle = 60;
    public static double hoodOffset = -6.5;
    public static double testHoodAngle = 30;

    //Turret Variables:
    public static double tKP = 0.015;
    public static double tKI = 0;
    public static double tKD = 0.0009;
    public static double tKV = 0;
    public static double tKS = 0.008;
    public static double turretTolerance = 1.5;
    public enum tState {off, tracking, aiming, aimed}
    public tState turretState = tState.off;
    private double turretTargetAngle, angleError, turretInput;
    public boolean turretAngleLimited = false;
    public boolean manualControl = false;

    //Absolute Encoder Variables:
    public static double absOffset = -170;
    double outputVoltage=0;
    double outputAngle=0;
    double totalRawAngle = 0;
    double turnCounter = 0;
    double turretAngle = 0;
    double previousAngle = 0;
    double previousRawAngle = 0;

    //Intake Variables:
    private double distance1, distance2, distance3;
    private double intakeSpeed = 1.0;
    private double rejectSpeed = -.4;
    public enum iState {idle, intaking, loading, rejecting, stuckBall}
    public iState intakeState = iState.idle;

    @Override
    public void runOpMode() {

        //Init various tools(?)
        follower= Constants.createFollower(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        initHardware();
        createLUTS();
        while (opModeInInit()) {
            //team/position selection
            if (gamepad1.left_stick_button) currentTeam = PremierAuto.teamEnum.blue;
            if (gamepad1.right_stick_button) currentTeam = PremierAuto.teamEnum.red;
            if (gamepad1.left_bumper) isFarZone = false;
            if (gamepad2.right_bumper) isFarZone = true;
            telemetryM.addLine("Left Stick = blue");
            telemetryM.addLine("Right Stick = red");
            telemetryM.addData("Team:",currentTeam);
            telemetryM.addLine("Right bumper = far, left = close");
            telemetryM.addData("Is Far:", isFarZone);
            telemetryM.update(telemetry);
        }

        onStart();

        while (opModeIsActive()) {
            follower.update();
            currentPose=follower.getPose();
            hubs.forEach(LynxModule::clearBulkCache);
            time.start();
            driverInput();
            drive.driveRobotCentric(strafeInput,driveInput,turnInput);
            sensorUpdate();
            turretUpdate();
            launcherUpdate();
            intakeUpdate();
            time.end();
            telemetry();

        }
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

        //color1 = hardwareMap.get(ColorSensor.class,"colorSensor");
        dist1 = hardwareMap.get(DistanceSensor.class, "distanceSensor1");
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
//        velLUT.add(95,1210);
        velLUT.add(95,1320);
        velLUT.add(105,1360);
        velLUT.add(115,1420);
        velLUT.add(125,1460);
        velLUT.add(135,1500);
        velLUT.add(145,1540);
        velLUT.add(155,1580);
        velLUT.add(maxRange,1620);
//        velLUT.add(105,1240);
//        velLUT.add(115,1300);
//        velLUT.add(125,1360);
//        velLUT.add(135,1400);
//        velLUT.add(145,1440);
//        velLUT.add(155,1480);
//        velLUT.add(maxRange,1540);
        velLUT.createLUT();

        angleLUT.add(minRange,30);
        angleLUT.add(45,30);
        angleLUT.add(55,32);
        angleLUT.add(65,34);
        angleLUT.add(75,36);
        angleLUT.add(85,37);
//        angleLUT.add(95,38);
        angleLUT.add(95,48);
        angleLUT.add(105,48);
        angleLUT.add(115,48);
        angleLUT.add(125,48);
        angleLUT.add(135,48);
        angleLUT.add(145,48);
        angleLUT.add(155,48);
        angleLUT.add(maxRange,48);
//        angleLUT.add(105,39);
//        angleLUT.add(115,40);
//        angleLUT.add(125,41);
//        angleLUT.add(135,42);
//        angleLUT.add(145,44);
//        angleLUT.add(155,46);
//        angleLUT.add(maxRange,47);
        angleLUT.createLUT();
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
        telemetryM.addData("Turncounter",turnCounter);
        telemetryM.addData("X",currentPose.getX());
        telemetryM.addData("Y",currentPose.getY());
        telemetryM.addData("Heading",Math.toDegrees(currentPose.getHeading()));
        telemetryM.addData("Range",launchRange);
        telemetryM.addData("Looptime (hz)", time.getHz());
        telemetryM.addData("Dist (1)",distance1);
        telemetryM.addData("Dist (2)",distance2);
        telemetryM.addData("Dist (3)",distance3);
        telemetryM.addData("Ballin1",ballIn1);
        telemetryM.addData("Ballin2",ballIn2);
        telemetryM.addData("Ballin3",ballIn3);
        telemetryM.update(telemetry);

        //final needs to highlight zone preference, should take from end of auto as baseline
    }

    public void driverInput() {
        //Drivetrain
        strafeInput = gamepad1.left_stick_x;
        driveInput = -gamepad1.left_stick_y;
        turnInput = gamepad1.right_stick_x * 0.7;
        if (Math.abs(driveInput) > 0.7 && Math.abs(strafeInput) < 0.3) strafeInput = 0;
        if (gamepad1.left_bumper) drive.setMaxSpeed(0.7);
        else drive.setMaxSpeed(1);

        //Intake
        if (gamepad1.right_trigger > 0.3) {
            intakeState = iState.intaking;
        } else if (gamepad1.b) intakeState = iState.rejecting;
        else if (gamepad1.right_bumper) {
            intakeState = iState.stuckBall;
            if (!rejectToggle) intakeTimer.reset();
            rejectToggle = true;
        }
        else {
            intakeState = iState.idle;
            rejectToggle = false;
        }


        //Launcher
        if (gamepad2.right_trigger > 0.2) {
            intakeState = iState.loading;
        }
        if (gamepad1.left_trigger > 0.2) {
            if (!blockerToggle) blockerTimer.reset();
            blockerToggle = true;
            if (turretState == tState.tracking) turretState = tState.aiming;
            if (launcherState != lState.firing) launcherState = lState.waiting;
            blockerState = bState.open;
        } else if (gamepad2.left_trigger > 0.2) {
            launcherState = lState.testing;
            blockerState = bState.open;
        } else {
            launcherState = lState.idle; //should be idle
            blockerState = bState.closed;
            if (turretState != tState.off) turretState = tState.tracking;
            blockerToggle = false;
        }

        if (gamepad1.left_stick_button) isFarZone = true;
        if (gamepad1.right_stick_button) isFarZone = false;
        if (gamepad1.x) {
            turretState = tState.off;
            launcherState = lState.off;
            blockerState = bState.disabled;
        }
        if (gamepad1.y) {
            turretState = tState.tracking;
            launcherState = lState.idle;
        }
//        if (gamepad1.left_trigger > 0.2 && launcherState != lState.waiting) {
//            launcherState = lState.firing;
//        } else if (gamepad1.left_trigger < 0) {
//            launcherState = lState.idle;
//            if (intakeState==iState.loading) intakeState = iState.idle;
//        }


        //Turret
        if (gamepad1.dpad_left) {
            turretDisabled = false;
            turretTargetAngle = 45;
        } else if (gamepad1.dpad_right) {
            turretDisabled = false;
            turretTargetAngle = -45;
        } else {
            turretDisabled = true;
        }
        if (Math.abs(gamepad2.left_stick_x) > 0) {
            manualControl = true;
            turretDisabled = true;
            turretInput = gamepad2.left_stick_x;
        } else {
            manualControl = false;
            turretDisabled = false;
        }

        if (gamepad2.a) blockerState = bState.closed;
        if (gamepad2.b) blockerState = bState.open;
    }

    public void sensorUpdate() {
        loopCount += 1;
        loopCount = loopCount % 9;
        switch (loopCount) {
            case 0:
                distance3 = dist3.getDistance(DistanceUnit.INCH);
                ballIn3 = distance3 < 3;
                break;
            case 3:
                distance2 = dist2.getDistance(DistanceUnit.INCH);
                ballIn2 = distance2 < 3;
                break;
            case 6:
                distance1 = dist1.getDistance(DistanceUnit.INCH);
                ballIn1 = distance1 < 3;
                break;
        }

    }

    public void turretUpdate() {
        //Absolute Encoder
        double turnCounterChange = 0;
        previousAngle = outputAngle;
        previousRawAngle = totalRawAngle;
        outputVoltage = absEncoder.getVoltage();
        outputAngle = outputVoltage / 5 * 360;
        if (Math.abs(outputAngle-previousAngle) > 180) {
            if (outputAngle < previousAngle) turnCounterChange = 1;
            else if (outputAngle > previousAngle) turnCounterChange = -1;
        }
        double tempTotalRawAngle = outputAngle + (turnCounter + turnCounterChange)*360;
        if (Math.abs(previousRawAngle - tempTotalRawAngle) > 180) turnCounterChange = 0;
        turnCounter += turnCounterChange;
        totalRawAngle = outputAngle + turnCounter*360;
        turretAngle = ((totalRawAngle + absOffset)/2);


        //Turret Position Logic
        if (turretState != tState.off){
            turretTargetAngle = calculateTurretAngle(currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
            turretTargetAngle = limitAngle(turretTargetAngle);
        }
        tController.setPIDF(tKP,tKI,tKD,tKV,tKS,voltage);
        if (!manualControl) turretInput = tController.calculate(turretAngle, turretTargetAngle);
        if (turretDisabled && !manualControl) turretInput = 0;
        if (Math.abs(tController.getPositionError()) < turretTolerance && turretState == tState.aiming) turretState = tState.aimed;
        else if (turretState == tState.aimed && Math.abs(tController.getPositionError()) > turretTolerance) turretState = tState.aiming;
        //if (tController.atSetPoint() && !manualControl) turretInput=0;
        //mTurret.set(turretInput);

    }

    public void launcherUpdate() {
        goalPose = new Pose(goalX,goalY);
        //Launch logic outline:
        //FSM - Idle state, waiting for turret state, active launch state with checks for extreme movement, sensor check to finish
        //Make a driver 2 toggle between close and far, assuming usually close play,
        //so prep speed reasonable if needing to go far for balls
        if (isFiring) isFiring=false;
        switch (launcherState) {
            case off:
                launcherTargetSpeed = 0;
                break;
            case idle:
                if (isFarZone) {
                    launcherTargetSpeed = farIdleSpeed;
                    //may need to make it only spin up if too low to maintain, not decrease suddenly, preventing undue power use
                    //assuming launch position is roughly the same -- maybe even making it copy previous launch position?
                    //maybe have somehting to begin adjusting once it comes back into launch zone - checking 4 corners & edges w/ trig
                } else {
                    launcherTargetSpeed = closeIdleSpeed;
                }
                //test idle mode: averages idle speed with last launch speed
//                if ((launcher.getVelocity() - launcherTargetSpeed) < 200 && (launcher.getVelocity() - launcherTargetSpeed) > 20) {
//                    launcherTargetSpeed = 0;
//                }
               // launcherTargetSpeed = .5 * (launcherTargetSpeed + lastLaunchSpeed); //assumes tempo, also getting rid of y
                break;
            case firing:
                isFiring=true;
                if (blockerTimer.seconds() > 0.2 && !gamepad1.right_bumper) intakeState = iState.loading;
                else if (!gamepad1.right_bumper) intakeState = iState.idle;
                launcherTargetSpeedTemp = velLUTClipped(launchRange);
                lastLaunchSpeed = launcherTargetSpeed;
                if (launcher.getVelocity() > launcherTargetSpeedTemp + 20) launcherTargetSpeed=0;
                else launcherTargetSpeed=5000; //sufficiently high to get max power response
                if (turretState == tState.aiming) launcherState = lState.waiting;
                break;
            case waiting:
                launcherTargetSpeed = velLUTClipped(launchRange);
                if (intakeState == iState.loading) intakeState = iState.idle;
                if (turretState == tState.aimed && follower.getVelocity().getMagnitude() < 3 && (Math.abs(launcherTargetSpeed-launcher.getVelocity())) < 40) launcherState = lState.firing;
                lastLaunchSpeed = launcherTargetSpeed;
                break;
            case testing:
                launcherTargetSpeed = testLaunchSpeed;
                //launcherTargetSpeed = velLUTClipped(launchRange);
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

    public double velLUTClipped(double input) {
        if (input < minRange) return velLUT.get(minRange+0.1);
        else if (input > maxRange) return velLUT.get(maxRange-0.1);
        else return velLUT.get(input);
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
                if (!ballIn1 || !ballIn2 || !ballIn3) {
                    mIntake.set(intakeSpeed);
                } else {
                    mIntake.set(0);
                }
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
                //might need to add the thing where intake runs backwards and then reverses quickly to launch third
            case stuckBall:
                if (intakeTimer.seconds() < 0.10) mIntake.set(-1);
                else mIntake.set(1);
                break;
        }
    }

    public void onStart() {
        if (!isFarZone) {
            startPose = new Pose(x(30), 135, a(270));
        } else {
            startPose = new Pose(x(54),7,a(90));
        }
        // need to update with logic for carrying over auto
        outputVoltage = absEncoder.getVoltage();
        outputAngle = outputVoltage / 5 * 360;
        turretState = tState.tracking;
        goalPose = new Pose(goalX,goalY);
        follower.setStartingPose(startPose);
        time = new LoopTimer();
    }

    public double calculateTurretAngle(double botX, double botY, double botHeading) {
        double goalX = goalPose.getX();
        double goalY = goalPose.getY();
        launchRange = Math.hypot(goalX-botX,goalY-botY);
        if (botY > 130) {
            goalY -= 5;
            goalX = x(0);
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
        if ((angleGoal > 150 && turretAngle < 0) || (angleGoal < -150 && turretAngle > 0)) {
            turretAngleLimited = true;
            return turretAngle;
        }
        if (angleGoal > 140) {
            angleGoal = 140;
            turretAngleLimited = true;
        } else if (angleGoal < -140) {
            angleGoal = -140;
            turretAngleLimited = true;
        }
        return angleGoal;
    }

    public double x(double input) {
        if (currentTeam == PremierAuto.teamEnum.blue) return input;
        else return 144-input;
    }

    public double a(double input) {
        if (currentTeam == PremierAuto.teamEnum.blue) return Math.toRadians(input);
        else return Math.toRadians(-(input-90)+90);
    }
}
