package org.firstinspires.ftc.teamcode.regionals;

import static org.firstinspires.ftc.teamcode.meet2.Meet2Auto.shooterSpeedGap;
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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.meet3.Meet3Auto;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.Objects;


@Autonomous(name = "Regional Auto", group = "Autonomous")
@Configurable // Panels
@SuppressWarnings("FieldCanBeLocal") // Stop Android Studio from bugging about variables being predefined
public class RegionalsAuto extends LinearOpMode {

    //Telemetry Manager

    private boolean isFull = false;


    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    //April Tag Variables

    //Pedropathing Variables
    public Pose currentPose;

    public final ElapsedTime launchTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private final ElapsedTime intakeTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private final ElapsedTime launchPrepTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public MotorEx fL, fR, bL, bR, launcher1, launcher2, turret, intake;
    public ServoEx hoodServo, blockerServo;
    private DistanceSensor distanceSensor1, distanceSensor2;
    private ColorSensor colorSensor;
    private String DTState = "drive", intakeState = "idle", turretState = "idle", launcherState = "idle", hoodState = "idle";
    public boolean DTisReady, intakeisReady, turretisReady, launcherisReady;
    public MotorGroup launcher;
    private ElapsedTime teleTimer;
    private MecanumDrive drive;


    //Team Dependents
    public static String team = "blue";
    private double goalID = 20;
    public Pose goalPose;
    private Pose aprilTagPose;
    private Pose poseResetPose;
    private boolean useRealStart = false;
    public static Pose endPose;
    //Lookup Tables
    public InterpLUT velocityLUT = new InterpLUT(), rangeLUT = new InterpLUT(), hoodLUT = new InterpLUT();

    //DT Variables
    private double driveInput, strafeInput, turnInput;
    public double driveAngleDegrees = 0;

    //Launcher Test Variables
    public static double speedOverride = 0;
    public static double angleOverride = 0;

    //Launcher Variables
    public static double kp = 1.5;
    public static double ki = 200;
    public static double kd = 0;
    public double launcherTargetVelocity;
    public static double launcherTestSpeed = 0.65;
    public double odoRange = 0;
    public boolean isIdle = true;

    //Hood Variables
    public double hoodTargetAngle = 40;
    public static double minHoodAngle = 33;
    public static double maxHoodAngle = 57;
    public static double hoodOffset = -1.5;

    //Blocker Variables
    public double closedAngle = 155;
    public double openAngle = 215;

    //Intake Variables
    public static double intakePickupSpeed = 1.0;
    public double transferLoadSpeed = 1.0;
    public static double intakeRejectSpeed = -0.5;

    //Turret Variables
    private PIDFController turretPIDF;
    private PIDController launcherPID;
    public static double turretTolerance = 2;
    public static double tkP = 0.0045;
    public static double tkI = 0;
    public static double tkD = 0.0003;
    public static double tkSCustom = 0.1;
    public static double errorTotal = 30;
    public double turretTargetPos;
    public double angleError;
    public boolean turretManualControl = false;
    private boolean turretAngleLimited = false;
    private double turretDriftOffset = 0;
    private boolean driftAdjustToggle = false;

    private double distance1;
    private double distance2;
    private double colorAlpha;

    private int cyclesSinceUpdate;

    //Sensor Variables
    public boolean ballIn1, ballIn2, ballIn3, prevBallIn1, prevBallIn2, prevBallIn3, prev2BallIn1, prev2BallIn2, prev2BallIn3;
    boolean sensorsEnabled = true;

    private double voltage;
    private double voltageMultiplier;
    //Timer

    AnalogInput absEncoder;
    public static double zeroPositionOffset = -173;
    double outputVoltage = 0;
    double outputAngle = 0;
    double totalRawAngle = 0;
    double turnCounter = 0;
    double trueTurretAngle = 0;
    double previousAngle = 0;
    double previousRawAngle = 0;

    double hoodCompensationAngle = 0;

    private final ElapsedTime autoTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private final ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    //Panels Editable Variables
    public static double intakeMaxPower = .7;
    public static double rampMaxPower = .7;
    public static double gateWaitTime = 0.8;
    public static double intakeMaxPower15 = 0.9;
    public static double rampMaxPower15 = 1.0;
    public static double farLaunchTime = 1.2;
    public static Pose endPosition = new Pose(0, 0, 0);


    //Auto Refactor Paths n such
    public PathChain ParkClose;
    boolean ParkInitialized = false;

    public static Pose scorePose;


    //PedroPathing PathChains
    public Pose startPose;
    public PathChain ScoreToRamp1;
    public PathChain ScoreToRamp2;
    public PathChain IntakeRamp;
    public PathChain ScoreToGPPClose;
    public PathChain PGPToScoreClose;
    public PathChain GPPIntakeClose;
    public PathChain closeStartToScore;
    public PathChain ScoreToPGPClose;
    public PathChain PGPIntakeClose;
    public PathChain PGPToGateClose;
    public PathChain CompatibleGateScore1;
    public PathChain CompatibleGateScore2;
    public PathChain RampToScore;
    public PathChain PPGIntakeClose;
    public PathChain PPGToScoreClose;
    public PathChain GPPToScoreClose;
    public PathChain FarCompatible_StartToGPP;
    public PathChain FarCompatible_IntakeGPP;
    public PathChain FarCompatible_GPPToLaunch;
    public PathChain FarCompatible_LaunchToLowerGPG;
    public PathChain FarCompatible_IntakeLowerGPG;
    public PathChain FarCompatible_LowerGPGToLaunch;
    public PathChain FarCompatible_LaunchToGate;
    public PathChain FarCompatible_IntakeGate;
    public PathChain FarCompatible_GateToLaunch;
    public PathChain FarCompatible_StartToPreload;
    public PathChain PPGToParkClose;

    //Changing variables
    public int autoState = 0;
    public int segmentState = 0;


    //Launcher Auto Variables
    public static double launchTime = .7;
    public double preparedLauncherVelocity = 0;

    //Turret Auto Variables
    public double preparedTargetPos = 0;

    public enum selectedAuto {
        Close15Compatible,
        Close15Solo,
        Far15Compatible,
        Far15Solo
    }

    public selectedAuto auto;


    @Override
    public void runOpMode() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // Panels telemetry
        LoopTimer timer = new LoopTimer();
        initMotors(); //Initializes subsystem motors
        createLUTs(); //Initialize lookup tables
        VoltageSensor voltageSensor;
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        voltage = voltageSensor.getVoltage();

        //voltageMultiplier = ((voltage-12)/1.5);
        voltageMultiplier = 1 - ((13.0 - voltage) / 13.0) * 2;
        if (voltageMultiplier > 1) {
            voltageMultiplier = 1;
        }
        if (voltageMultiplier < .8) {
            voltageMultiplier = .8;
        }

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL)); //Bulk read to reduce loop time

        //Initialize PP Follower
        follower = Constants.createFollower(hardwareMap);
        while (opModeInInit()) {
            telemetry.addLine("Driver: Press left stick for blue team and right stick for red team");
            telemetry.addLine("A = Close Solo. B = Far Solo. X = Close Compatible. Y = Far Compatible");
            telemetry.addData("Team Selected:", team);
            telemetry.addData("Auto Selected:", auto);
            if (auto == selectedAuto.Close15Compatible || auto == selectedAuto.Far15Compatible)
                telemetry.addLine("THIS IS ALLIANCE COMPATIBLE");
            telemetry.update();
            if (gamepad1.left_stick_button) {
                team = "blue";
            } else if (gamepad1.right_stick_button) {
                team = "red";
            }
            if (gamepad1.a) auto = selectedAuto.Close15Solo;
            if (gamepad1.x) auto = selectedAuto.Close15Compatible;
            if (gamepad1.b) auto = selectedAuto.Far15Solo;
            if (gamepad1.y) auto = selectedAuto.Far15Compatible;
        }
        updateTeamDependents();
        follower.setStartingPose(startPose);

        buildPaths();


        launchTimer.reset();
        autoTimer.reset();
        if (opModeIsActive()) {
            previousAngle = absEncoder.getVoltage() / 5 * 360;
            while (opModeIsActive()) {

                hubs.forEach(LynxModule::clearBulkCache); //Bulk reading


                //Update Pedro follower and Panels
                follower.update();
                currentPose = follower.getPose();

                if (auto == selectedAuto.Close15Compatible) Close15Compatible();
                if (auto == selectedAuto.Close15Solo) Close15Solo();
                if (auto == selectedAuto.Far15Compatible) Far15Compatible();
                if (auto == selectedAuto.Far15Solo) Far15Solo();


                teleop();

                telemetryUpdate();
                telemetryM.update(telemetry);
                endPosition = currentPose;
            }
        }
        endPosition = currentPose;
    }

    public void telemetryUpdate() {
        telemetryM.addLine("Robot Position");
        telemetryM.addData("X", currentPose.getX());
        telemetryM.addData("Y", currentPose.getY());
        telemetryM.addData("Heading", Math.toDegrees(currentPose.getHeading()));
        telemetryM.addData("Goal Distance", odoRange);
        telemetryM.addLine("Launcher Info");
        telemetryM.addData("Launcher Velocity", launcher.getVelocity());
        telemetryM.addData("Launcher Target", velocityLUT.get(launcherTargetVelocity));
        telemetryM.addData("Launcher Power", launcher.get());
        telemetryM.addLine("Turret Info");
        telemetryM.addData("Turret Target", turretTargetPos);
        telemetryM.addData("Turret Pos", turret.getCurrentPosition());
        telemetryM.addData("Turret Angle", turret.getCurrentPosition() * 360 / 1076.6);
        telemetryM.addData("Turret Motor Power", turret.get());
        //telemetryM.addData("Distance",testDistanceSensor.getDistance(DistanceUnit.INCH));
    }

    public void initMotors() {
        fL = new MotorEx(hardwareMap, "fL", Motor.GoBILDA.RPM_312);
        fR = new MotorEx(hardwareMap, "fR", Motor.GoBILDA.RPM_312);
        bL = new MotorEx(hardwareMap, "bL", Motor.GoBILDA.RPM_312);
        bR = new MotorEx(hardwareMap, "bR", Motor.GoBILDA.RPM_312);
        launcher1 = new MotorEx(hardwareMap, "launcherMotor1", Motor.GoBILDA.BARE);
        launcher2 = new MotorEx(hardwareMap, "launcherMotor2", Motor.GoBILDA.BARE);
        intake = new MotorEx(hardwareMap, "intakeMotor", Motor.GoBILDA.BARE);
        turret = new MotorEx(hardwareMap, "turretMotor", Motor.GoBILDA.RPM_435);
        //turret.resetEncoder();
        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turret.setRunMode(Motor.RunMode.RawPower);
        turret.setCachingTolerance(0.01);
        launcher2.setInverted(true);
        launcher1.setVeloCoefficients(kp, ki, kd);
        launcher2.setVeloCoefficients(kp, ki, kd);
        //launcher1.setFeedforwardCoefficients(ks,kv);
        //launcher2.setFeedforwardCoefficients(ks,kv);
        launcher1.setRunMode(Motor.RunMode.VelocityControl);
        launcher2.setRunMode(Motor.RunMode.VelocityControl);
        drive = new MecanumDrive(fL, fR, bL, bR);
        drive.setRightSideInverted(false);
        turretPIDF = new PIDFController(tkP, tkI, tkD, 0);
        turretPIDF.setIntegrationBounds(-errorTotal, errorTotal);
        //launcherPID = new PIDController(kp,ki,kd);
        launcher = new MotorGroup(launcher1, launcher2);
        distanceSensor1 = hardwareMap.get(DistanceSensor.class, "firstDistanceSensor");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "thirdDistanceSensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        hoodServo = new ServoEx(hardwareMap, "hoodServo", 0, 300);
        hoodServo.setPwm(new PwmControl.PwmRange(500, 2500));
        hoodServo.setInverted(true);
        blockerServo = new ServoEx(hardwareMap, "blockerServo", 0, 300);
        blockerServo.setPwm(new PwmControl.PwmRange(500, 2500));
        absEncoder = hardwareMap.get(AnalogInput.class, "AbsoluteEncoder");
    }

    public void createLUTs() {
        //create shooting speed lookup table

        //Add values (obtained empirically)
        //Input is distance, output is shooter velocity
        rangeLUT.add(47, 0.41);
        rangeLUT.add(55, 0.42);
        rangeLUT.add(65, 0.45);
        rangeLUT.add(75, 0.48);
        rangeLUT.add(85, .485);
        rangeLUT.add(95, 0.5);
        rangeLUT.add(106, 0.52);
        rangeLUT.add(126, .56);
        rangeLUT.add(147, .605);
        rangeLUT.add(160, 0.645);
        rangeLUT.add(170,0.68);

        rangeLUT.createLUT();

        velocityLUT.add(-1.001, 2500);
        velocityLUT.add(-0.8, 2000);
        velocityLUT.add(-0.6, 1500);
        velocityLUT.add(-.4, 1000);
        velocityLUT.add(-.2, 500);
        velocityLUT.add(0, 0);
        velocityLUT.add(0.2, 500);
        velocityLUT.add(0.4, 1000);
        velocityLUT.add(0.6, 1500);
        velocityLUT.add(0.8, 2000);
        velocityLUT.add(1.001, 2500);
        velocityLUT.createLUT();


        hoodLUT.add(0, 33);
        hoodLUT.add(20, 33);
        hoodLUT.add(47, 33);
        hoodLUT.add(55, 33);
        hoodLUT.add(65, 35);
        hoodLUT.add(75, 36);
        hoodLUT.add(85, 39);
        hoodLUT.add(95, 40);
        hoodLUT.add(106, 42);
        hoodLUT.add(126, 44);
        hoodLUT.add(147, 47);
        hoodLUT.add(160, 48.5);
        hoodLUT.add(170, 48.5);

        hoodLUT.createLUT();
    }

    public void teleop() {
        if (sensorsEnabled) sensorTeleOp();
        absEncoderTeleOp();
        DTTeleOp();
        turretTeleOp();
        launcherTeleOp();
        intakeTeleOp();
        hoodTeleOp();
    }

    public void absEncoderTeleOp() {
        outputVoltage = absEncoder.getVoltage();
        outputAngle = outputVoltage / 5 * 360;
        if (Math.abs(outputAngle - previousAngle) < 180) {
        } else if (outputAngle < previousAngle) turnCounter += 1;
        else if (outputAngle > previousAngle) turnCounter -= 1;
        totalRawAngle = outputAngle + turnCounter * 360;
        trueTurretAngle = ((totalRawAngle + zeroPositionOffset) / 2);
        previousAngle = outputAngle;
        previousRawAngle = totalRawAngle;
    }

//    public double calculateHoodOffset(double VocityError) {
//        return Math.abs(velocityError) * angleVelocityMultiplier;
//    }

    public void hoodTeleOp() {
        switch (hoodState) {
            case "idle":
                hoodServo.set(hoodToServoAngle(hoodTargetAngle));
                break;
            case "up":
                hoodTargetAngle = maxHoodAngle;
                hoodServo.set(hoodToServoAngle(hoodTargetAngle));
                break;
            case "down":
                hoodTargetAngle = minHoodAngle;
                hoodServo.set(hoodToServoAngle(hoodTargetAngle));
                break;
            case "adjusting":
                hoodTargetAngle = hoodToServoAngle(angleLUT(odoRange));
                hoodServo.set(hoodTargetAngle);
                break;
        }
        telemetryM.addData("Hood Angle", hoodTargetAngle);
        telemetryM.addData("Servo Angle", hoodServo.get());
    }

    public double servoToHoodAngle(double servoInput) {
        return (servoInput) * 13 / 121 + minHoodAngle + hoodOffset;
    }

    public double hoodToServoAngle(double hoodAngle) {
        return (hoodAngle - minHoodAngle - hoodOffset) * 121 / 13;
    }

    public double angleLUT(double range) {
        if (range > 20 && range < 170) return hoodLUT.get(range);
        else if (range < 20) return hoodLUT.get(21);
        else return hoodLUT.get(169);
    }

    public void sensorTeleOp() {
        if (Objects.equals(launcherState,"idle") || Objects.equals(launcherState, "off")) {
            if (cyclesSinceUpdate == 9) {
                cyclesSinceUpdate = 0;
                distance1 = distanceSensor1.getDistance(DistanceUnit.INCH);
                prevBallIn1 = ballIn1;
                ballIn1 = distance1 < 4;
            }
            if (cyclesSinceUpdate == 6) {
                colorAlpha = colorSensor.alpha();
                prevBallIn2 = ballIn2;
                ballIn2 = colorAlpha > 70;
            }
            if (cyclesSinceUpdate == 3) {
                distance2 = distanceSensor2.getDistance(DistanceUnit.INCH);
                prevBallIn3 = ballIn3;
                ballIn3 = distance2 < 4;
            }
            cyclesSinceUpdate++;
        }

        telemetryM.addData("Distance1", distance1);
        telemetryM.addData("Distance2", distance2);
        telemetryM.addData("Ball in 1?", ballIn1);
        telemetryM.addData("Ball in 2?", ballIn2);
        telemetryM.addData("Ball in 3?", ballIn3);
    }


    public void DTTeleOp() {

        switch (DTState) {
            case "idle":
                DTisReady = true;
                break;
            case "drive":
                if (Math.abs(driveInput) < .1) {
                    driveInput = 0;
                }
                if (Math.abs(strafeInput) < .1) {
                    strafeInput = 0;
                }
        }
    }

    public void intakeTeleOp() {
        switch (intakeState) {
            case "idle":
                intakeisReady = true;
                intake.stopMotor();
                blockerServo.set(openAngle);
                break;
            case "firing":
//                if (Math.abs(intake.getVelocity()) < 300) intake.set(1.0);
//                else intake.set(transferLoadSpeed);
                intake.set(transferLoadSpeed);
                blockerServo.set(openAngle);
                break;
            case "beginIntaking":
                intakeTimer.reset();
                blockerServo.set(closedAngle);
                intakeState = "intaking";
            case "intaking":
                if ((!ballIn1 && !ballIn2 && !ballIn3 && sensorsEnabled) || intakeTimer.seconds() > 0.3) {
                    intake.set(intakePickupSpeed);
                    blockerServo.set(closedAngle);
                    if (ballIn1 && ballIn2 && ballIn3 && prevBallIn1 && prevBallIn2 && prevBallIn3) {
                        intakeState = "idle";
                        //launcherState = "preparing";
                        turretState = "tracking";
                    }
                }
                break;
            case "rejecting":
                intake.set(intakeRejectSpeed);
                blockerServo.set(openAngle);
                break;
            case "shortReject":
                intake.set(-.8);
                intakeTimer.reset();
                intakeState = "shortRejecting";
                break;
            case "shortRejecting":
                if (intakeTimer.seconds() > 0.08) {
                    intake.set(1);
                }
                break;
        }
    }

    public void turretTeleOp() {

        switch (turretState) {
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
        if (!turretManualControl) turretControlLoop();
    }

    public void launcherTeleOp() {
        double botX = currentPose.getX();
        double botY = currentPose.getY();
        double goalX = goalPose.getX();
        double goalY = goalPose.getY();
        switch (launcherState) {
            case "idle":
                launcherisReady = true;
                isIdle = true;
                launcher.stopMotor();
                odoRange = Math.hypot(goalX - botX, goalY - botY);
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
                intakeState = "idle";
                launcherState = "aiming";
                turretState = "aiming";
                hoodState = "adjusting";
                break;
            case "aiming":
                launcherSpinUp();
                launchTimer.reset();
                launcher.set(launcherTargetVelocity);
                intakeState = "idle";
                if (Math.abs(launcher.getVelocity()) > Math.abs(velocityLUT.get(launcherTargetVelocity)) - shooterSpeedGap) {
                    launcherState = "acc_ready";
                }
                break;
            case "acc_ready":
                launcherSpinUp();
                launcher.set(launcherTargetVelocity);
                launchTimer.reset();
                if (Objects.equals(turretState, "aimed") && follower.getVelocity().getMagnitude() < 6 && Math.abs(follower.getAngularVelocity()) < 0.5) {
                    intakeState = "firing";
                    launcherState = "firing";
                    break;
                }
            case "firing":
                launcherSpinUp();
                //launcher.set(launcherTargetVelocity);
                if (launcher.getVelocity() > velocityLUT.get(launcherTargetVelocity)) {
                    launcher.stopMotor();
                } else launcher.set(1);
                if (Objects.equals(turretState, "aiming") ||
                        (follower.getVelocity().getMagnitude() > 6 && Math.abs(follower.getAngularVelocity()) < 0.5)) {
                    intakeState = "idle";
                    launcherState = "acc_ready";
                }
//                if (Math.abs(launcher.getVelocity()) > Math.abs(velocityLUT.get(launcherTargetVelocity)) - 35 && odoRange > 100) {
//                    intakeState = "idle";
//                    launcherState = "aiming";
//                }
                break;
            case "prepare":
                launchPrepTimer.reset();
                launcherState = "preparing";
                break;
            case "preparing":
                launcher.set(preparedLauncherVelocity);
                break;
            case "rejecting":
                launcherTargetVelocity = -.3;
                launcher.set(launcherTargetVelocity);
                launcherisReady = true;
                break;
        }
    }

    public double calculateRangeLUT(double input) {
        if (input < 47) {
            transferLoadSpeed = 0;
            return rangeLUT.get(48);
        } else if (input > 170) {
            transferLoadSpeed = 0;
            return rangeLUT.get(169);
        } else {
            if (currentPose.getY() > 50) transferLoadSpeed = 0.8;
            else transferLoadSpeed = 0.55;
            return rangeLUT.get(input);
        }
    }

    public void launcherSpinUp() {
        double botX = currentPose.getX();
        double botY = currentPose.getY();
        double goalX = goalPose.getX();
        double goalY = goalPose.getY();
        odoRange = Math.hypot(goalX - botX, goalY - botY);
        launcherTargetVelocity = calculateRangeLUT(odoRange);
    }

    public void turretTargetProcedure() {
        turretTrackProcedure();
        if (!turretAngleLimited) angleError = turretTargetPos - trueTurretAngle;
        else angleError = 100;
    }

    public void turretTrackProcedure() {
        double turretAngle;
        turretAngle = calculateTurretAngle(currentPose.getX(), currentPose.getY(), Math.toDegrees((currentPose.getHeading())));
        turretAngle = turretAngleLimiter(turretAngle);

        //if (follower.getVelocity().getMagnitude() < 2 && Math.abs(follower.getAngularVelocity()) < 0.2)
        turretTargetPos = turretAngle;
    }

    public double calculateTurretAngle(double botX, double botY, double botHeading) {
        double goalX = goalPose.getX();
        double goalY = goalPose.getY();
        if (botY > 125) {
            goalY = 139;
        } else if (botY < 50) {
            goalX = x(5);
        }
        double targetAngle = Math.toDegrees(Math.atan2(goalY - botY, goalX - botX));
        targetAngle -= botHeading + 180 + turretDriftOffset;
        telemetryM.addData("Target Angle", targetAngle);
        return targetAngle;
    }

    public void turretControlLoop() {
        double output = turretPIDF.calculate(trueTurretAngle, turretTargetPos);
        double kSFriction;
        kSFriction = tkSCustom * (Math.abs(turretPIDF.getPositionError()) / turretPIDF.getPositionError());
        if (Math.abs(turretPIDF.getPositionError()) > 1) output += kSFriction;
        else if (Math.abs(turretPIDF.getPositionError()) < 1) output = 0;
        if (Math.abs(output) > 0.7) output = output / Math.abs(output) * 0.7;
        turret.set(output);
    }

    public double turretAngleLimiter(double angleAttempt) {
        double realAngle = angleAttempt;
        if (realAngle > 180) {
            realAngle -= 360;
        } else if (realAngle < -180) {
            realAngle += 360;
        }
        if ((realAngle > 170 && trueTurretAngle < 0) || (realAngle < -170 && trueTurretAngle > 0)) {
            turretAngleLimited = true;
            return turretTargetPos;
        } else turretAngleLimited = false;
        if (realAngle > 175) {
            turretAngleLimited = true;
            realAngle = 175;
        } else if (realAngle < -145) {
            realAngle = -145;
            turretAngleLimited = true;
        }
        return realAngle;
    }

    public void updateTeamDependents() {
        if (Objects.equals(team, "blue")) {
            goalID = 20;
            if (auto == selectedAuto.Far15Solo || auto == selectedAuto.Far15Compatible) {
                startPose = new Pose(63, 8, Math.toRadians(90));

            }
            else {
                startPose = new Pose(30, 135, Math.toRadians(270));
            }
            goalPose = new Pose(0, 144, Math.toRadians(135));
            poseResetPose = new Pose(114, 7, Math.toRadians(90)); //need to find good one
            aprilTagPose = new Pose(15, 130, 0);
        } else if (Objects.equals(team, "red")) {
            goalID = 24;
            if (auto == selectedAuto.Far15Solo || auto == selectedAuto.Far15Compatible) {
                startPose = new Pose(x(63), 8, a(90));
            }
            else {
                startPose = new Pose(x(30), 135, a(270));

            }
            goalPose = new Pose(144, 144, Math.toRadians(45));
            poseResetPose = new Pose(30, 7, Math.toRadians(90)); //need to find good one
            aprilTagPose = new Pose(129, 130, 0);
        }
    }

    public double x(double X) { //transform X based on team
        if (Objects.equals(team, "blue")) return X;
        else return 141.5 - (X);
    }

    public double a(double angle) { //transform heading based on team
        if (Objects.equals(team, "blue")) return Math.toRadians(angle);
        else return Math.toRadians(-(angle - 90) + 90);
    }

    public void premoveTurret(double x, double y, double heading) {
        double turretAngle;
        turretAngle = calculateTurretAngle(x, y, heading);
        turretAngle = turretAngleLimiter(turretAngle);
        preparedTargetPos = turretAngle;
        double goalX = goalPose.getX();
        double goalY = goalPose.getY();
        odoRange = Math.hypot(goalX - x, goalY - y);
        preparedLauncherVelocity = calculateRangeLUT(odoRange);
    }

    public void resetSubsystems() {
        launcherState = "idle";
        intakeState = "idle";
        turretState = "idle";
    }

    public void intakeBalls() {
        //launcherState = "rejecting";
        intakeState = "beginIntaking";
    }

    public void launchBalls() {
        launcherState = "beginLaunchSequence";
    }

    Runnable preSpinLauncher = new Runnable() {
        @Override
        public void run() {
            launcherState = "preparing";
            turretState = "tracking";
        }
    };

    public void Close15Solo() {
        switch (autoState) {
            case 0:
                PreLoadClose();
                if (segmentState == -1) {
                    segmentState = 0;
                    autoState = 1;
                }
                break;
            case 1:
                PGPClose(false);
                if (segmentState == -1) {
                    segmentState = 0;
                    autoState = 2;
                }
                break;
            case 2:
                gateClose();
                if (segmentState == -1) {
                    segmentState = 0;
                    autoState = 3;
                }
                break;
            case 3:
                PPGClose(false);
                if (segmentState == -1) {
                    segmentState = 0;
                    autoState = 4;
                }
                break;
            case 4:
                GPPClose();
                if (segmentState == -1) {
                    segmentState = 0;
                    autoState = 6;
                }
                break;
            case 5:
                parkClose(follower.getPose());
                if (segmentState == -1) {
                    segmentState = 0;
                    autoState = 6;
                }
                break;
            case 6:
                resetSubsystems();
                break;
        }
    }

    public void Close15Compatible() {
        switch (autoState) {
            case 0:
                PreLoadClose();
                if (segmentState == -1) {
                    segmentState = 0;
                    autoState = 1;
                }
                break;
            case 1:
                PGPClose(true);
                if (segmentState == -1) {
                    segmentState = 0;
                    autoState = 2;
                }
                break;
            case 2:
                gateClose();
                if (segmentState == -1) {
                    segmentState = 0;
                    autoState = 3;
                }
                break;
            case 3:
                gateClose();
                if (segmentState == -1) {
                    segmentState = 0;
                    autoState = 4;
                }
                break;
            case 4:
                PPGClose(true);
                if (segmentState == -1) {
                    segmentState = 0;
                    autoState = 6;
                }
                break;
            case 6:
                resetSubsystems();
                break;
        }
    }

    public void Far15Solo() {
        switch (autoState) {
            case 0:
                PreLoadFar();
                if (segmentState == -1) {
                    segmentState = 0;
                    autoState = 1;
                }
                break;
            case 1:
                GPPFar();
                if (segmentState == -1) {
                    segmentState = 0;
                    autoState = 2;
                }
                break;
            case 2:
                HumanPlayerFar();
                if (segmentState == -1) {
                    segmentState = 0;
                    autoState = 3;
                }
                break;
            case 3:
                HumanPlayerFar();
                if (segmentState == -1) {
                    segmentState = 0;
                    autoState = 4;
                }
                break;
            case 4:
                PPGFar();
                if (segmentState == -1) {
                    segmentState = 0;
                    autoState = 5;
                }
                break;
            case 5:
                ParkFar(follower.getPose());
                if (segmentState == -1) {
                    segmentState = 0;
                    autoState = 6;
                }
                break;
            case 6:
                resetSubsystems();
                break;
        }
    }

    public void Far15Compatible() {
        switch (autoState) {
            case 0:
                PreLoadFar();
                if (segmentState == -1) {
                    segmentState = 0;
                    autoState = 1;
                }
                break;
            case 1:
                GPPFar();
                if (segmentState == -1) {
                    segmentState = 0;
                    autoState = 2;
                }
                break;
            case 2:
                HumanPlayerFar();
                if (segmentState == -1) {
                    segmentState = 0;
                    autoState = 3;
                }
                break;
            case 3:
                HumanPlayerFar();
                if (segmentState == -1) {
                    segmentState = 0;
                    autoState = 4;
                }
                break;
            case 4:
                HumanPlayerFar();
                if (segmentState == -1) {
                    segmentState = 0;
                    autoState = 5;
                }
                break;
            case 5:
                ParkFar(follower.getPose());
                if (segmentState == -1) {
                    segmentState = 0;
                    autoState = 6;
                }
                break;
            case 6:
                resetSubsystems();
                break;
        }
    }

    public void PreLoadClose() {
        switch (segmentState) {
            case 0:
                follower.followPath(closeStartToScore);
                hoodState = "adjusting";
                premoveTurret(closeStartToScore.endPose().getX(), closeStartToScore.endPose().getY(), Math.toDegrees(closeStartToScore.endPose().getHeading()));
                segmentState = 1;
                break;
            case 1:
                if (!follower.isBusy() && follower.getVelocity().getMagnitude() < 2.0) {
                    launchBalls();
                    segmentState = 2;
                }
                break;
            case 2:
                if (launchTimer.seconds() > launchTime) {
                    resetSubsystems();
                    segmentState = -1;
                }
                break;
        }
    }

    public void PPGClose(boolean parkBeforeShoot) {
        switch (segmentState) {
            case 0:
                follower.followPath(PPGIntakeClose, intakeMaxPower, true);
                intakeBalls();
                segmentState = 1;
                break;
            case 1:
                if (follower.atParametricEnd()) {
                    segmentState = 2;
                    if (!parkBeforeShoot) {
                        segmentState = 2;
                        resetSubsystems();
                        premoveTurret(PPGToScoreClose.endPose().getX(), PPGToScoreClose.endPose().getY(), Math.toDegrees(PPGToScoreClose.endPose().getHeading()));
                        follower.followPath(PPGToScoreClose);
                    } else {
                        resetSubsystems();
                        premoveTurret(PPGToScoreClose.endPose().getX(), PPGToScoreClose.endPose().getY(), Math.toDegrees(PPGToScoreClose.endPose().getHeading()));
                        follower.followPath(PPGToParkClose);
                        segmentState = 2;
                    }
                }
                break;
            case 2:
                if (!follower.isBusy() && follower.getVelocity().getMagnitude() < 2.0) {
                    launchBalls();
                    segmentState = 3;
                }
                break;
            case 3:
                if (launchTimer.seconds() > launchTime) {
                    resetSubsystems();
                    segmentState = -1;
                }
                break;
        }
    }

    public void PGPClose(boolean openGate) {
        switch (segmentState) {
            case 0:
                follower.followPath(ScoreToPGPClose);
                segmentState = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(PGPIntakeClose, intakeMaxPower, true);
                    intakeBalls();
                    segmentState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    resetSubsystems();
                    if (openGate) {
                        follower.followPath(PGPToGateClose);
                        segmentState = 3;
                    } else {
                        follower.followPath(PGPToScoreClose);
                        premoveTurret(PGPToScoreClose.endPose().getX(), PGPToScoreClose.endPose().getY(), Math.toDegrees(PGPToScoreClose.endPose().getHeading()));
                        segmentState = 5;
                    }
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(CompatibleGateScore1);
                    segmentState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    premoveTurret(CompatibleGateScore2.endPose().getX(), CompatibleGateScore2.endPose().getY(), Math.toDegrees(CompatibleGateScore2.endPose().getHeading()));
                    follower.followPath(CompatibleGateScore2);
                    segmentState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy() && follower.getVelocity().getMagnitude() < 2.0) {
                    launchBalls();
                    segmentState = 6;
                }
                break;
            case 6:
                if (launchTimer.seconds() > launchTime) {
                    resetSubsystems();
                    segmentState = -1;
                }
                break;
        }
    }

    public void gateClose() {
        switch (segmentState) {
            case 0:
                follower.followPath(ScoreToRamp1);
                segmentState = 1;
                break;
            case 1:
                if (follower.atParametricEnd()) {
                    follower.followPath(ScoreToRamp2, rampMaxPower, true);
                    waitTimer.reset();
                    segmentState = 2;
                }
                break;
            case 2:
                if (follower.atParametricEnd() || waitTimer.seconds() > 1) {
                    waitTimer.reset();
                    intakeBalls();
                    segmentState = 3;
                }
                break;
            case 3:
                if (waitTimer.seconds() > 0.6) {
                    follower.followPath(IntakeRamp);
                    segmentState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    waitTimer.reset();
                    segmentState = 5;
                }
                break;
            case 5:
                if (waitTimer.seconds() > gateWaitTime || isFull) {
                    resetSubsystems();
                    premoveTurret(RampToScore.endPose().getX(), RampToScore.endPose().getY(), Math.toDegrees(RampToScore.endPose().getHeading()));
                    follower.followPath(RampToScore);
                    segmentState = 6;
                }
                break;
            case 6:
                if (!follower.isBusy() && follower.getVelocity().getMagnitude() < 2.0) {
                    launchBalls();
                    segmentState = 7;
                }
                break;
            case 7:
                if (launchTimer.seconds() > launchTime) {
                    resetSubsystems();
                    segmentState = -1;
                }
        }
    }

    public void GPPClose() {
        switch (segmentState) {
            case 0:
                follower.followPath(ScoreToGPPClose);
                segmentState = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(GPPIntakeClose, intakeMaxPower, true);
                    intakeBalls();
                    segmentState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    resetSubsystems();
                    follower.followPath(GPPToScoreClose);
                    premoveTurret(GPPToScoreClose.endPose().getX(), GPPToScoreClose.endPose().getY(), Math.toDegrees(GPPToScoreClose.endPose().getHeading()));
                    segmentState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy() && follower.getVelocity().getMagnitude() < 2.0) {
                    launchBalls();
                    segmentState = 4;
                }
                break;
            case 4:
                if (launchTimer.seconds() > launchTime) {
                    resetSubsystems();
                    segmentState = -1;
                }
                break;
        }
    }

    public void parkClose(Pose initialPose) {
        if (!ParkInitialized) {
            ParkClose = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(initialPose.getX(), initialPose.getY(), initialPose.getHeading()),
                                    new Pose(x(48), 70, initialPose.getHeading()))
                    )
                    .setConstantHeadingInterpolation(initialPose.getHeading())
                    .build();
            ParkInitialized = true;
        }
        switch (segmentState) {
            case 0:
                follower.followPath(ParkClose);
                segmentState = 1;
                break;
            case 1:
                if (!follower.isBusy() && follower.getVelocity().getMagnitude() < 2.0) {
                    launchBalls();
                    segmentState = 2;
                }
                break;
            case 2:
                if (launchTimer.seconds() > launchTime) {
                    resetSubsystems();
                    segmentState = -1;
                }
                break;
        }
    }

    public void PreLoadFar() {
        switch (segmentState) {
            case 0:
                follower.followPath(FarCompatible_StartToPreload);
                hoodState = "adjusting";
                premoveTurret(currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
                segmentState = 10;
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.turnTo(a(150));
                    segmentState = 1;
                }
                break;
            case 1:
                if (!follower.isBusy() && follower.getVelocity().getMagnitude() < 1) {
                    launchBalls();
                    launchTimer.reset();
                    segmentState = 2;
                }
                break;
            case 2:
                if (launchTimer.seconds() > farLaunchTime - 0.5) {
                    intakeState = "shortReject";
                }
                if (launchTimer.seconds() > farLaunchTime) {
                    resetSubsystems();
                    segmentState = -1;
                }
                break;
        }
    }

    public void GPPFar() {
        switch (segmentState) {
            case 0:
                follower.followPath(FarCompatible_StartToGPP);
                segmentState = 1;
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(FarCompatible_IntakeGPP, intakeMaxPower, true);
                    intakeBalls();
                    segmentState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(FarCompatible_GPPToLaunch);
                    premoveTurret(FarCompatible_GPPToLaunch.endPose().getX(), FarCompatible_GPPToLaunch.endPose().getY(), Math.toDegrees(FarCompatible_GPPToLaunch.endPose().getHeading()));
                    segmentState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy() && follower.getVelocity().getMagnitude() < 2.0) {
                    launchBalls();
                    segmentState = 4;
                }
                break;
            case 4:
                if (launchTimer.seconds() > launchTime) {
                    resetSubsystems();
                    segmentState = -1;
                }
                break;
        }
    }

    public void HumanPlayerFar() {
        switch (segmentState) {
            case 0:
                follower.followPath(FarCompatible_LaunchToLowerGPG);
                segmentState = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(FarCompatible_IntakeLowerGPG, intakeMaxPower, true);
                    intakeBalls();
                    segmentState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(FarCompatible_LowerGPGToLaunch);
                    premoveTurret(FarCompatible_LowerGPGToLaunch.endPose().getX(), FarCompatible_LowerGPGToLaunch.endPose().getY(), Math.toDegrees(FarCompatible_LowerGPGToLaunch.endPose().getHeading()));
                    segmentState = -1;
                }
                break;
            case 3:
                if (!follower.isBusy() && follower.getVelocity().getMagnitude() < 2.0) {
                    launchBalls();
                    segmentState = 4;
                }
                break;
            case 4:
                if (launchTimer.seconds() > launchTime) {
                    resetSubsystems();
                    segmentState = -1;
                }
                break;
        }
    }

    public void HumanPlayerBlindFar() {
        switch (segmentState) {
            case 0:
                follower.followPath(FarCompatible_LaunchToGate);
                segmentState = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(FarCompatible_IntakeGate, intakeMaxPower, true);
                    intakeBalls();
                    segmentState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(FarCompatible_GateToLaunch);
                    premoveTurret(FarCompatible_GateToLaunch.endPose().getX(), FarCompatible_GateToLaunch.endPose().getY(), Math.toDegrees(FarCompatible_GateToLaunch.endPose().getHeading()));
                    segmentState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy() && follower.getVelocity().getMagnitude() < 2.0) {
                    launchBalls();
                    segmentState = 4;
                }
                break;
            case 4:
                if (launchTimer.seconds() > launchTime) {
                    resetSubsystems();
                    segmentState = -1;
                }
                break;
        }
    }

    public void PGPFar() {

    }

    public void PPGFar() {

    }


    public void ParkFar(Pose initialPose) {
        switch (segmentState) {
            case 0:

                break;
            case 1:

                break;
            case 2:

                break;
            case 3:

                break;
            case 4:

                break;
            case 5:

                break;
        }
    }

    public void buildPaths() {

        scorePose = new Pose(x(57),82);

        closeStartToScore = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(30), 136), new Pose(x(45), 92))
                )
                .setTangentHeadingInterpolation()
                .addTemporalCallback(300, preSpinLauncher)
                .build();

        PPGIntakeClose = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(scorePose.getX(),84), new Pose(x(21), 84))
                )
                .setTangentHeadingInterpolation()
                .build();

        PPGToScoreClose = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(21),84),scorePose)
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addTemporalCallback(300, preSpinLauncher)
                .build();

        PPGToParkClose = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(21),84),new Pose (x(60),104))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addTemporalCallback(300, preSpinLauncher)
                .build();


        ScoreToPGPClose = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(45), 92), new Pose(x(45), 60))
                )
                .setLinearHeadingInterpolation(a(289), a(180), 0.8)
                .build();

        PGPIntakeClose = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(45), 60), new Pose(x(19), 60))
                )
                .setTangentHeadingInterpolation()
                .build();

        PGPToGateClose = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(x(19), 60),
                                new Pose(x(27), 70),
                                new Pose(x(17.75), 70)
                        )
                )
                .setConstantHeadingInterpolation(a(180))
                .build();

        PGPToScoreClose = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(19), 60), scorePose)
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addTemporalCallback(300, preSpinLauncher)
                .build();

        CompatibleGateScore1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(x(17.75), 70),
                                new Pose(x(57), 70)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        CompatibleGateScore2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(x(57), 70),
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(a(180), a(225), 0.8)
                .addTemporalCallback(300, preSpinLauncher)
                .build();

        ScoreToRamp1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(scorePose, new Pose(x(32), 62))
                )
                .setLinearHeadingInterpolation(a(225), a(135), 0.8)
                //.setNoDeceleration()
                .build();

        ScoreToRamp2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(x(32), 62),
                                new Pose(x(12.5), 62)
                        )
                )
                .setConstantHeadingInterpolation(a(135))
                .build();

        IntakeRamp = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(x(12.5), 62),
                                new Pose(x(12.5), 58)
                        )
                )
                .setConstantHeadingInterpolation(a(135))
                .build();

        RampToScore = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(12.5), 52), scorePose)
                )
                //.setConstantHeadingInterpolation(a(155))
                .setTangentHeadingInterpolation()
                .setReversed()
                .addTemporalCallback(300, preSpinLauncher)
                .build();

        ScoreToGPPClose = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(scorePose, new Pose(x(52), 36))
                )
                .setConstantHeadingInterpolation(a(180))
                .build();

        GPPIntakeClose = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(52), 36), new Pose(x(20), 36))
                )
                .setTangentHeadingInterpolation()
                .build();

        GPPToScoreClose = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(20), 36), new Pose(x(60), 104))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addTemporalCallback(300, preSpinLauncher)
                .build();
        FarCompatible_StartToPreload = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(x(63), 8),
                                new Pose(x(63), 11)
                        )
                )
                .setConstantHeadingInterpolation(a(90))
                .build();
        FarCompatible_StartToGPP = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(x(63), 12),
                                new Pose(x(40), 36)
                        )
                )
                .setLinearHeadingInterpolation(a(150), a(180))
                .build();
        FarCompatible_IntakeGPP = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(x(40), 36),
                                new Pose(x(10), 36)
                        )
                )
                .setConstantHeadingInterpolation(a(180))
                .build();
        FarCompatible_GPPToLaunch = follower
                .pathBuilder()
                .addPath(
                    new BezierLine(
                            new Pose(x(10), 35),
                            new Pose(x(63), 12)
                    )
                )
                .setConstantHeadingInterpolation(a(180))
                .build();
        FarCompatible_LaunchToLowerGPG = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(x(65), 12),
                                new Pose(x(7), 30)
                        )
                )
                .setLinearHeadingInterpolation(a(180), a(270))
                .build();
        FarCompatible_IntakeLowerGPG = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(x(7),30),
                                new Pose(x(7),10)
                        )
                )
                .setLinearHeadingInterpolation(a(270),a(270))
                .build();
        FarCompatible_LowerGPGToLaunch = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(x(7),10),
                                new Pose(x(63), 12)
                        )
                )
                .setLinearHeadingInterpolation(a(270), a(180))
                .build();
        FarCompatible_LaunchToGate = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(x(65),12),
                                new Pose(x(10),38)
                        )
                )
                .setLinearHeadingInterpolation(a(180), a(120))
                .build();
        FarCompatible_IntakeGate = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(x(10),38),
                                new Pose(x(7),60)
                        )
                )
                .setLinearHeadingInterpolation(a(120), a(100))
                .build();
        FarCompatible_GateToLaunch = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(x(7),60),
                                new Pose(x(65),12)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }
}
