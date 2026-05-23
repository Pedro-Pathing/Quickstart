package org.firstinspires.ftc.teamcode.prototyping;

import static org.firstinspires.ftc.teamcode.meet2.Meet2Auto.shooterSpeedGap;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.utils.LoopTimer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Meet_ILT.Drawing;
import org.firstinspires.ftc.teamcode.Meet_ILT.ILT_Auto;
import org.firstinspires.ftc.teamcode.meet3.Meet3Auto;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;
import java.util.Objects;

@TeleOp(name = "Regionals Teleop")
@Configurable
@Disabled
public class deprecatedTeleOp extends LinearOpMode {
    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    //April Tag Variables
    private boolean isFull = false;

    //Pedropathing Variables
    private Pose currentPose;

    private final ElapsedTime launchTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    private MotorEx fL, fR, bL, bR, launcher1, launcher2, turret, intake;
    private ServoEx hoodServo;
    private DistanceSensor distanceSensor1, distanceSensor2;
    private ColorSensor colorSensor;
    private String DTState="drive", intakeState="idle", turretState="idle", launcherState="idle", hoodState="idle";
    private boolean DTisReady, intakeisReady, turretisReady, launcherisReady;
    private MotorGroup launcher;
    private ElapsedTime teleTimer;
    private MecanumDrive drive;


    //Team Dependents
    public static String team = Meet3Auto.team;
    private double goalID = 20;
    private Pose startPose;
    private Pose goalPose;
    private Pose aprilTagPose;
    private Pose poseResetPose;
    private boolean useRealStart = false;
    public static Pose endPose;
    //Lookup Tables
    private InterpLUT velocityLUT = new InterpLUT(), rangeLUT= new InterpLUT(), hoodLUT = new InterpLUT();

    //DT Variables
    private double driveInput, strafeInput, turnInput;
    private double driveAngleDegrees = 0;

    //Launcher Test Variables
    public static double speedOverride = 0;
    public static double angleOverride = 0;

    //Launcher Variables
    public static double kp = 1.5;
    public static double ki = 200;
    public static double kd = 0;
    private double launcherTargetVelocity;
    public static double launcherTestSpeed = 0.65;
    public double odoRange = 0;
    public boolean isIdle = true;

    //Hood Variables
    public double hoodTargetAngle = 40;
    public static double minHoodAngle = 33;
    public static double maxHoodAngle = 57;
    public static double hoodOffset = -1.5;

    //Blocker Variables
    public double closedAngle = 0;
    public double openAngle = 180;

    //Intake Variables
    public static double intakePickupSpeed = 1.0;
    public double transferLoadSpeed = 1.0;
    public static double intakeRejectSpeed = -0.5;

    //Turret Variables
    private PIDFController turretPIDF;
    private PIDController launcherPID;
    public static double turretTolerance = 2;
    public static double tkP = 0.0025;
    public static double tkI = 0;
    public static double tkD = 0.00005;
    public static double tkSCustom = 0.13;
    public static double errorTotal = 30;
    private int turretTargetPos;
    private double angleError;
    private boolean turretManualControl = false;
    private boolean turretAngleLimited = false;
    private double turretDriftOffset = 0;
    private boolean driftAdjustToggle = false;

    private double distance1=0;
    private double distance2=0;
    private double colorAlpha=0;

    private int cyclesSinceUpdate;

    //Sensor Variables
    boolean ballIn1, ballIn2, ballIn3, prevBallIn1, prevBallIn2, prevBallIn3;
    boolean sensorsEnabled = true;

    private double voltage;
    private double voltageMultiplier;

    double turret_x;
    double turret_y;

    //The variable to store our instance of the vision portal.

    private VisionPortal visionPortal;

    private Drawing drawing;

    @Override
    public void runOpMode() {
        drawing = new Drawing();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        LoopTimer time = new LoopTimer();

        //set the default settings for motors and initializes them (wow)
        initMotors();

        drawing.initilize();

        VoltageSensor voltageSensor;
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        voltage = voltageSensor.getVoltage();

        //voltageMultiplier = ((voltage-12)/1.5);
        voltageMultiplier = 1-((13.0-voltage)/13.0)*2;
        if (voltageMultiplier > 1) {
            voltageMultiplier = 1;
        }
        if (voltageMultiplier < .8) {
            voltageMultiplier = .8;
        }

        //initTrackingSoftware();
        createLUTs();

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        teleTimer = new ElapsedTime();
        follower= Constants.createFollower(hardwareMap);
        while (opModeInInit()) {
            telemetry.addLine("Driver: Press left stick for blue team and right stick for red team.");
            telemetry.addData("Team Selected:",team);
            telemetry.addData("Use Base start position(testing)",useRealStart);
            telemetry.update();
            if (gamepad1.left_stick_button) {
                team="blue";
            } else if (gamepad1.right_stick_button) {
                team="red";
            }
            if (gamepad1.a) {
                useRealStart = true;
            }
        }
        updateTeamDependents();
        if (ILT_Auto.endPosition.getX() != 0 && !useRealStart) {
            startPose = ILT_Auto.endPosition;
        } else {
            try {
                if (endPose.getX() != 0 && !useRealStart){
                    startPose = endPose;
                }
            } catch (NullPointerException ignored) {
            }
            try {
                if (startPose.getX() == 0 && useRealStart){
                    turret.resetEncoder();
                }
            } catch (NullPointerException ignored) {
            }
        }
        if (useRealStart) turret.resetEncoder();
        follower.setStartingPose(startPose);
        if (opModeIsActive()) {
            teleTimer.reset();
            while (opModeIsActive()) {
                time.start();
                hubs.forEach(LynxModule::clearBulkCache);
                follower.update();
                currentPose = follower.getPose();
                teleOp();


                telemetryUpdate();
                time.end();
                telemetryM.addData("looptime(hz)",time.getHz());
                telemetryM.update(telemetry);

            }
        }
        endPose = follower.getPose();

    }   // end method runOpMode()

    public void telemetryUpdate() {
        telemetryM.addLine("Robot Position");
        telemetryM.addData("X",currentPose.getX());
        telemetryM.addData("Y",currentPose.getY());
        telemetryM.addData("Heading",Math.toDegrees(currentPose.getHeading()));
        telemetryM.addData("Goal Distance",odoRange);
        telemetryM.addLine("Launcher Info");
        telemetryM.addData("Launcher Velocity",launcher.getVelocity());
        telemetryM.addData("Launcher Target",velocityLUT.get(launcherTargetVelocity));
        telemetryM.addData("Launcher Power",launcher.get());
        telemetryM.addLine("Turret Info");
        telemetryM.addData("Turret Target",turretTargetPos);
        telemetryM.addData("Turret Pos",turret.getCurrentPosition());
        telemetryM.addData("Turret Angle",turret.getCurrentPosition() * 360/1076.6);
        telemetryM.addData("Turret Motor Power",turret.get());
        //telemetryM.addData("Distance",testDistanceSensor.getDistance(DistanceUnit.INCH))
        drawing.drawRobot(new Position(DistanceUnit.INCH, currentPose.getX(),currentPose.getY(), 0,0), new YawPitchRollAngles(AngleUnit.RADIANS,turret.getCurrentPosition(), 0,0,0));
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
        //turret.resetEncoder();
        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turret.setRunMode(Motor.RunMode.RawPower);
        launcher2.setInverted(true);
        launcher1.setVeloCoefficients(kp,ki,kd);
        launcher2.setVeloCoefficients(kp,ki,kd);
        //launcher1.setFeedforwardCoefficients(ks,kv);
        //launcher2.setFeedforwardCoefficients(ks,kv);
        launcher1.setRunMode(Motor.RunMode.VelocityControl);
        launcher2.setRunMode(Motor.RunMode.VelocityControl);
        drive = new MecanumDrive(fL, fR, bL, bR);
        drive.setRightSideInverted(false);
        turretPIDF = new PIDFController(tkP, tkI, tkD, 0);
        turretPIDF.setIntegrationBounds(-errorTotal,errorTotal);
        //launcherPID = new PIDController(kp,ki,kd);
        launcher = new MotorGroup(launcher1, launcher2);
        distanceSensor1 = hardwareMap.get(DistanceSensor.class, "firstDistanceSensor");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "thirdDistanceSensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        hoodServo = new ServoEx(hardwareMap,"hoodServo",0, 300);
        hoodServo.setPwm(new PwmControl.PwmRange(500,2500));
        hoodServo.setInverted(true);
    }

    public void createLUTs() {
        //create shooting speed lookup table

        //Add values (obtained empirically)
        //Input is distance, output is shooter velocity
        if (speedOverride == 0) {
            rangeLUT.add(47,0.4);
            rangeLUT.add(55,0.41);
            rangeLUT.add(65,0.44);
            rangeLUT.add(75,0.47);
            rangeLUT.add(85,.47);
            rangeLUT.add(95,0.5);
            rangeLUT.add(106,0.52);
            rangeLUT.add(126,.56);
            rangeLUT.add(147,.6);
            rangeLUT.add(160,0.64);
        } else {
            rangeLUT.add(20,speedOverride);
            rangeLUT.add(160,speedOverride);
        }
        rangeLUT.createLUT();

        velocityLUT.add(-1.001,2500);
        velocityLUT.add(-0.8,2000);
        velocityLUT.add(-0.6,1500);
        velocityLUT.add(-.4,1000);
        velocityLUT.add(-.2,500);
        velocityLUT.add(0,0);
        velocityLUT.add(0.2,500);
        velocityLUT.add(0.4,1000);
        velocityLUT.add(0.6,1500);
        velocityLUT.add(0.8,2000);
        velocityLUT.add(1.001,2500);
        velocityLUT.createLUT();

        hoodLUT.add(0, 33);
        hoodLUT.add(20, 33);
        hoodLUT.add(47,33);
        hoodLUT.add(55,33);
        hoodLUT.add(65,35);
        hoodLUT.add(75,36);
        hoodLUT.add(85,39);
        hoodLUT.add(95,40);
        hoodLUT.add(106,42);
        hoodLUT.add(126, 44.5);
        hoodLUT.add(147, 47.5);
        hoodLUT.add(160,49);
        hoodLUT.createLUT();

    }

    public void teleOp() {
        DriverInput();
        if (sensorsEnabled) sensorTeleOp();
        DTTeleOp();
        turretTeleOp();
        launcherTeleOp();
        intakeTeleOp();
        hoodTeleOp();
    }

    public void hoodTeleOp() {
        //hoodTargetAngle = LaunchAngleCalculator.calcBestAngle(launcher.getVelocity(),odoRange, telemetryM);
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
                hoodServo.set(hoodToServoAngle(angleLUT(odoRange)));
                break;
            case "firing":
                hoodServo.set(hoodToServoAngle(hoodTargetAngle));
                break;

        }
        telemetryM.addData("Hood Angle",hoodTargetAngle);
        //telemetryM.addData("Servo Angle",hoodServo.get());
    }

    public double servoToHoodAngle(double servoInput) {
        return (servoInput) * 13 / 121 + minHoodAngle + hoodOffset;
    }

    public double hoodToServoAngle(double hoodAngle) {
        return (hoodAngle-minHoodAngle - hoodOffset) * 121 / 13;
    }

    public double angleLUT(double range) {
        if (range > 20 && range < 160) return hoodLUT.get(range);
        else return 50;
    }
    public void DriverInput() {

//        if (gamepad1.dpad_up) {
//            hoodState = "up";
//        } else if (gamepad1.dpad_down) {
//            hoodState = "down";
//        } else if (gamepad1.dpad_left) {
//            hoodState = "idle";
//        }

        //DT Section
        strafeInput = gamepad1.left_stick_x;
        driveInput = -gamepad1.left_stick_y;
        turnInput = gamepad1.right_stick_x;
        if (gamepad1.left_bumper) {
            drive.setMaxSpeed(0.7);
            turnInput *= (0.5/0.7);
        } else {
            if (Math.abs(driveInput) > 0.7 && Math.abs(strafeInput) < 0.3) strafeInput = 0;
            drive.setMaxSpeed(1);
        }

        if (gamepad1.right_trigger > 0.4 && gamepad1.left_trigger > 0.4) {
            follower.setPose(poseResetPose);
        }
        if (gamepad2.right_trigger > 0.4 && gamepad2.left_trigger > 0.4) {
            follower.setPose(new Pose(139.5 - poseResetPose.getX(),poseResetPose.getY(),poseResetPose.getHeading()));
        }
        if (gamepad2.right_stick_button && gamepad2.left_bumper) turret.resetEncoder();

        if (gamepad1.left_stick_button) turretTargetPos = 0;

        //Intake Section
        if (gamepad2.a || gamepad1.a) {
            intakeState = "intaking";
            launcherState = "rejecting";
        } else if (gamepad2.y || gamepad1.dpad_down) {
            intakeState = "rejecting";
            launcherState = "rejecting";
        } else if (gamepad2.b || gamepad1.b) {
            intakeState = "idle";
            if (Objects.equals(launcherState,"rejecting")) {
                launcherState = "idle";
            }
        } else if (gamepad1.right_stick_button) {
            intakeState = "firing";
        } else if (gamepad1.x || gamepad2.x) {
            intakeState = "idle";
            turretState = "idle";
            launcherState = "idle";
            hoodState = "adjusting";
            //hoodState = "idle";
        }
        if (gamepad1.y) {
            intakeState = "idle";
            turretState = "tracking";
            hoodState = "adjusting";
//            launcherState = "preparing";
        }

        //Launcher Section
        if (gamepad2.dpad_up) {
            turretDriftOffset = 0;
        } else if (gamepad2.dpad_down) {
            sensorsEnabled = false;
            ballIn1 = false;
            ballIn2 = false;
            ballIn3 = false;
        }
        if (gamepad1.right_bumper || gamepad2.right_bumper) {
            launcherState = "beginLaunchSequence";
            turretState = "aiming";
            intakeState = "idle";
            launchTimer.reset();
        }

        //Turret Section
        if (Math.abs(gamepad2.left_stick_x) > 0.1 || gamepad2.left_bumper) {
            turretManualControl = true;
            if (turret.getCurrentPosition() > turretAngleToTicks(-135) && gamepad2.left_stick_x < 0) {
                turret.set(gamepad2.left_stick_x * .25);
            } else if (turret.getCurrentPosition() < turretAngleToTicks(135) && gamepad2.left_stick_x > 0) {
                turret.set(gamepad2.left_stick_x * .25);                                                                 //////    '                                                                                                                                     iiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiki88888888888888888888888888888888888888888888888888888888888888888888888///////////////////////////');
            } else {
                turret.set(0);
            }
        } else turretManualControl = false;
        /*if (gamepad2.dpad_left) { // test
            turretTargetPos = turretAngleToTicks(45);
        } else if (gamepad2.dpad_right) {
            turretTargetPos = turretAngleToTicks(-45);
        } else if (gamepad2.left_stick_button) {
            turretTargetPos = turretAngleToTicks(0);
        }*/
        if (gamepad2.dpad_left && driftAdjustToggle) { // in case turret drifts, player 2 clicks left or right for autoaim adjustment!!
            driftAdjustToggle = false;
            turretDriftOffset -= 1;
        } else if (gamepad2.dpad_right && driftAdjustToggle) {
            driftAdjustToggle = false;
            turretDriftOffset += 1;
        } else {
            driftAdjustToggle = true;
        }

        driveAngleDegrees = Math.toDegrees(currentPose.getHeading());
    }

    public void sensorTeleOp() {
        if(cyclesSinceUpdate == 3) {
            distance1 = distanceSensor1.getDistance(DistanceUnit.INCH);
            prevBallIn1 = ballIn1;
            ballIn1 = distance1 < 4;
        }
        if(cyclesSinceUpdate == 6) {
            colorAlpha = colorSensor.alpha();
            prevBallIn2 = ballIn2;
            ballIn2 = colorAlpha > 70;
        }
        if(cyclesSinceUpdate == 9) {
            distance2 = distanceSensor2.getDistance(DistanceUnit.INCH);
            cyclesSinceUpdate = 0;
            prevBallIn3 = ballIn3;
            ballIn3 = distance2 < 4;
        }
        cyclesSinceUpdate++;

        telemetryM.addData("Distance1", distance1);
        telemetryM.addData("Distance2", distance2);
        telemetryM.addData("Ball in 1?", ballIn1);
        telemetryM.addData("Ball in 2?", ballIn2);
        telemetryM.addData("Ball in 3?", ballIn3);
    }


    public void DTTeleOp() {

        switch (DTState){
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
                drive.driveRobotCentric(strafeInput,driveInput,turnInput);
        }
    }

    public void intakeTeleOp() {
        switch (intakeState){
            case "idle":
                intakeisReady = true;
                intake.stopMotor();
                break;
            case "firing":
//                if (Math.abs(intake.getVelocity()) < 300) intake.set(1.0);
//                else intake.set(transferLoadSpeed);
                intake.set(transferLoadSpeed);
                break;
            case "intaking":
                intake.set(intakePickupSpeed);
                if (ballIn1 && ballIn2 && ballIn3 && prevBallIn1 && prevBallIn2 && prevBallIn3) {
                    intakeState = "idle";
                    //launcherState = "preparing";
                    turretState = "tracking";
                }
                break;
            case "rejecting":
                intake.set(intakeRejectSpeed);
                break;
        }
    }

    public void turretTeleOp() {

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
                break;
        }
        if (!turretManualControl) turretControlLoop();
    }

    public void launcherTeleOp() {
        double botX = currentPose.getX();
        double botY = currentPose.getY();
        double goalX = goalPose.getX();
        double goalY = goalPose.getY();
        switch (launcherState){
            case "idle":
                launcherisReady = true;
                isIdle = true;
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
                intakeState = "idle";
                launcherState = "aiming";
                turretState = "aiming";
                break;
            case "aiming":
                launcherSpinUp();
                launcher.set(launcherTargetVelocity);
                intakeState = "idle";
                if (Math.abs(launcher.getVelocity()) > Math.abs(velocityLUT.get(launcherTargetVelocity)) - 100) {
                    hoodState = "firing";
                }
                if (Math.abs(launcher.getVelocity()) > Math.abs(velocityLUT.get(launcherTargetVelocity)) - shooterSpeedGap) {
                    launcherState = "acc_ready";
                }
                break;
            case "acc_ready":
                launcherSpinUp();
                launcher.set(launcherTargetVelocity);
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
                if (Objects.equals(turretState,"aiming") || (follower.getVelocity().getMagnitude() > 6 && Math.abs(follower.getAngularVelocity()) < 0.5)) {
                    intakeState = "idle";
                    launcherState = "acc_ready";
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

    public double calculateRangeLUT(double input) {
        if (input < 47) {
            transferLoadSpeed = 0;
            return rangeLUT.get(48);
        } else if (input > 160) {
            transferLoadSpeed = 0;
            return rangeLUT.get(159);
        } else {
            if (70 < input && input < 90) transferLoadSpeed = 0.85 * voltageMultiplier;
            else if (90 < input && input < 110) transferLoadSpeed = 0.7 * voltageMultiplier;
            else if (input > 110) transferLoadSpeed = 0.55 * voltageMultiplier;
            else transferLoadSpeed = 1;
            return rangeLUT.get(input);
        }
    }

    public void launcherSpinUp() {
        double botX = currentPose.getX();
        double botY = currentPose.getY();
        double goalX = goalPose.getX();
        double goalY = goalPose.getY();
        odoRange = Math.hypot(goalX-turret_x,goalY-turret_y);
        launcherTargetVelocity = calculateRangeLUT(odoRange);
    }

    public void turretTargetProcedure() {
        turretTrackProcedure();
        if (!turretAngleLimited) angleError = turretTicksToAngle(turretTargetPos-turret.getCurrentPosition());
        else angleError = 100;
    }

    public void turretTrackProcedure() {
        double turretAngle;
        turretAngle = calculateTurretAngle(currentPose.getX(), currentPose.getY(), Math.toDegrees((currentPose.getHeading())));
        turretAngle = turretAngleLimiter(turretAngle);
        //if (follower.getVelocity().getMagnitude() < 2 && Math.abs(follower.getAngularVelocity()) < 0.2)
        turretTargetPos = turretAngleToTicks(turretAngle);
    }

    public double calculateTurretAngle(double botX, double botY, double botHeading) {
        double goalX = goalPose.getX();
        double goalY = goalPose.getY();
        //double turret_offset_y = .430688;
        double offsetMagnitude = -1;
        turret_x = botX + offsetMagnitude * Math.cos(Math.toRadians(botHeading));
        turret_y = botY + offsetMagnitude * Math.sin(Math.toRadians(botHeading));
        double targetAngle = Math.toDegrees(Math.atan2(goalY-botY,goalX-botX));
        targetAngle -= botHeading + 180 + turretDriftOffset;
        telemetryM.addData("Target Angle",targetAngle);
        return targetAngle;
    }

    public void turretControlLoop() {
        double output = turretPIDF.calculate(turret.getCurrentPosition(),turretTargetPos);
        double kSFriction;
        if (Math.abs(turretPIDF.getPositionError()) <= turretTolerance) {
            turret.stopMotor();
        }
        else {
            kSFriction = tkSCustom * (Math.abs(turretPIDF.getPositionError()) / turretPIDF.getPositionError());
            output += kSFriction;
            turret.set(output);
        }
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
        if ((realAngle > 175 && turret.getCurrentPosition() < 0) || (realAngle < -160 && turret.getCurrentPosition() > 0)) {
            turretAngleLimited = true;
            return turretTicksToAngle(turretTargetPos);
        } else turretAngleLimited = false;
        if (realAngle > 160) {
            turretAngleLimited = true;
            realAngle = 160;
        } else if (realAngle < -135) {
            realAngle = -135;
            turretAngleLimited = true;
        }
        return realAngle;
    }

    public void updateTeamDependents() {
        if (Objects.equals(team,"blue")){
            goalID = 20;
            startPose = new Pose(30,136,Math.toRadians(270));
            goalPose = new Pose(0,144,Math.toRadians(135));
            poseResetPose = new Pose(114,7,Math.toRadians(90)); //need to find good one
            aprilTagPose = new Pose(15,130,0);
        } else if (Objects.equals(team,"red")) {
            goalID = 24;
            startPose = new Pose(x(30),136,a(270));
            goalPose = new Pose(144,144,Math.toRadians(45));
            poseResetPose = new Pose(30,7,Math.toRadians(90)); //need to find good one
            aprilTagPose = new Pose(129,130,0);
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
}