package org.firstinspires.ftc.teamcode.meet3;

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
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.meet3.Meet3Auto;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;
import java.util.Objects;

@TeleOp(name = "Meet 3 Teleop")
@Disabled
//@Configurable
public class Meet3Teleop extends LinearOpMode {

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    //April Tag Variables

    //Pedropathing Variables
    private Pose currentPose;

    private final ElapsedTime launchTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    private MotorEx fL, fR, bL, bR, launcher1, launcher2, turret, intake;
    private ServoEx hoodServo, blockerServo;
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
    //Lookup Tables
    private InterpLUT velocityLUT = new InterpLUT(), rangeLUT= new InterpLUT(), hoodLUT = new InterpLUT();

    //DT Variables
    private double driveInput, strafeInput, turnInput;
    private double driveAngleDegrees = 0;

    //Launcher Variables
    public static double kp = 1.5;
    public static double ki = 200;
    public static double kd = 0;
    private double launcherTargetVelocity;
    public static double launcherTestSpeed = 0.65;
    public double odoRange = 0;

    //Hood Variables
    public double hoodTargetAngle = 40;
    public static double minHoodAngle = 33;
    public static double maxHoodAngle = 55;

    //Intake Variables
    public static double intakePickupSpeed = 1.0;
    public double transferLoadSpeed = 1.0;
    public static double intakeRejectSpeed = -0.5;

    //Turret Variables
    private PIDFController turretPIDF;
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

    //Sensor Variables
    boolean ballIn1;
    boolean ballIn2;
    boolean ballIn3;


    //The variable to store our instance of the vision portal.

    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        LoopTimer timer = new LoopTimer();

        //set the default settings for motors and initializes them (wow)
        initMotors();
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
        if (Meet3Auto.endPosition.getX() != 0 && !useRealStart) {
            startPose = Meet3Auto.endPosition;
        } else {
            turret.resetEncoder();
        }
        follower.setStartingPose(startPose);
        if (opModeIsActive()) {
            teleTimer.reset();
            while (opModeIsActive()) {
                hubs.forEach(LynxModule::clearBulkCache);
                follower.update();
                currentPose = follower.getPose();
                teleOp();


                telemetryUpdate();
                telemetryM.update(telemetry);

            }
        }

    }   // end method runOpMode()

    public void telemetryUpdate() {
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
        //telemetryM.addData("Distance",testDistanceSensor.getDistance(DistanceUnit.INCH));
    }

    public void initMotors() {
        fL = new MotorEx(hardwareMap, "fL", Motor.GoBILDA.RPM_312);
        fR = new MotorEx(hardwareMap, "fR", Motor.GoBILDA.RPM_312);
        bL = new MotorEx(hardwareMap, "bL", Motor.GoBILDA.RPM_312);
        bR = new MotorEx(hardwareMap, "bR", Motor.GoBILDA.RPM_312);
        launcher1 = new MotorEx(hardwareMap, "launcherMotor1", Motor.GoBILDA.BARE);
        launcher2 = new MotorEx(hardwareMap, "launcherMotor2", Motor.GoBILDA.BARE);
        launcher1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        launcher2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        intake = new MotorEx(hardwareMap,"intakeMotor",Motor.GoBILDA.BARE);
        turret = new MotorEx(hardwareMap,"turretMotor",Motor.GoBILDA.RPM_435);
        //turret.resetEncoder();
        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turret.setRunMode(Motor.RunMode.RawPower);
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
//        rangeLUT.add(20,0.43);
//        rangeLUT.add(48,0.43);
//        rangeLUT.add(55,0.43);
//        rangeLUT.add(65.7,0.45);
//        rangeLUT.add(75,0.475);
//        rangeLUT.add(85,0.5);
//        rangeLUT.add(100,.53);
//        rangeLUT.add(110,0.55);
//        rangeLUT.add(115,0.57);
//        rangeLUT.add(123,0.67);
//        rangeLUT.add(130,0.67);
//        rangeLUT.add(140,0.74);
//        rangeLUT.add(160,0.79);
        rangeLUT.add(20,0.43);
        rangeLUT.add(48,0.44);
        rangeLUT.add(55,0.45);
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

        hoodLUT.add(20,33);
        hoodLUT.add(160,55);
        hoodLUT.createLUT();
    }

    public void teleOp() {
        DriverInput();
        sensorTeleOp();
        DTTeleOp();
        turretTeleOp();
        launcherTeleOp();
        intakeTeleOp();
        hoodTeleOp();
    }

    public void hoodTeleOp() {
        switch (hoodState) {
            case "idle":
                hoodServo.disable();
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
                hoodServo.set(hoodToServoAngle(hoodLUT.get(odoRange)));
                break;
        }
        telemetryM.addData("Hood Angle",hoodTargetAngle);
        telemetryM.addData("Servo Angle",hoodServo.get());
    }

    public double servoToHoodAngle(double servoInput) {
        return (servoInput) * 13 / 121 + minHoodAngle;
    }

    public double hoodToServoAngle(double hoodAngle) {
        return (hoodAngle-minHoodAngle) * 121 / 13;
    }

    public void DriverInput() {

        if (gamepad1.dpad_up) {
            hoodState = "up";
        } else if (gamepad1.dpad_down) {
            hoodState = "down";
        } else if (gamepad1.dpad_left) {
            hoodState = "idle";
        }

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

        if (gamepad2.right_trigger > 0.4 && gamepad2.left_trigger > 0.4) {
            follower.setPose(poseResetPose);
        }
        if (gamepad2.right_stick_button && gamepad2.left_bumper) turret.resetEncoder();

        if (gamepad1.left_stick_button) turretTargetPos = 0;

        //Intake Section
        if (gamepad2.a || gamepad1.a) {
            intakeState = "intaking";
            launcherState = "rejecting";
        } else if (gamepad2.y || gamepad1.y) {
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
        }

        //Launcher Section
        if (gamepad2.dpad_up) {
            launcherState = "testSpeed";
        } else if (gamepad2.dpad_down) {
            launcherState = "idle";
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
                turret.set(gamepad2.left_stick_x * .4);
            } else if (turret.getCurrentPosition() < turretAngleToTicks(135) && gamepad2.left_stick_x > 0) {
                turret.set(gamepad2.left_stick_x * .4);
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
        double distance1 = distanceSensor1.getDistance(DistanceUnit.INCH);
        double distance2 = distanceSensor2.getDistance(DistanceUnit.INCH);
        double colorAlpha = colorSensor.alpha();

        ballIn1 = distance1 < 4;
        ballIn2 = colorAlpha > 50;
        ballIn3 = distance2 < 4;

        telemetryM.addData("Distance1", distance1);
        telemetryM.addData("Distance2", distance2);
        telemetryM.addData("Alpha", colorAlpha);
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
                if (ballIn1 && ballIn2 && ballIn3) {
                    intakeState = "idle";
                    if (Objects.equals(launcherState,"rejecting")) {
                        launcherState = "idle";
                    }
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
                if (Objects.equals(turretState, "aimed") && follower.getVelocity().getMagnitude() < 6 && Math.abs(follower.getAngularVelocity()) < 0.5) {
                    intakeState = "firing";
                    launcherState = "firing";
                    break;
                }
            case "firing":
                launcherSpinUp();
                if (launcher.getVelocity() > velocityLUT.get(launcherTargetVelocity)) {
                    launcher.stopMotor();
                } else launcher.set(1);
                //launcher.set(launcherTargetVelocity);
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
        if (input < 20 || input > 160) {
            return launcherTestSpeed;
        } else {
            if (80 < input && input < 110) transferLoadSpeed = 0.8;
            else if (input > 110) transferLoadSpeed = 0.6;
            else transferLoadSpeed = 1;
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
        if ((realAngle > 160 && turret.getCurrentPosition() < 0) || (realAngle < -160 && turret.getCurrentPosition() > 0)) {
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
            startPose = new Pose(18.5,118.5,Math.toRadians(143+180));
            goalPose = new Pose(0,144,Math.toRadians(135));
            poseResetPose = new Pose(114,7,Math.toRadians(90)); //need to find good one
            aprilTagPose = new Pose(15,130,0);
        } else if (Objects.equals(team,"red")) {
            goalID = 24;
            startPose = new Pose(x(18.5),118.5,a(143+180));
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