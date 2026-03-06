package org.firstinspires.ftc.teamcode.Tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.panels.Panels;
import com.bylazar.utils.LoopTimer;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Timer;

@TeleOp
@Configurable
public class SystemTest extends LinearOpMode {

    private final Pose goalPos = new Pose(144, 144, Math.toRadians(45));

    public static boolean usePins = false;

    public static boolean turretAutoAim = false;

    public static int turretTargetPosition = 0;


    public static int shooterTargetVelocity = 0;



    public static double intakePower = 0;

    public static double transferPower = 0;

    public static double leftPinPositon = 0.575;
    public static double centerPinPosition = 0.105;
    public static double rightPinPosition = 0.275;

    public static double liftPos = 0.275;

    LoopTimer timer = new LoopTimer();




    @Override
    public void runOpMode() throws InterruptedException {


        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "motor_tm");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PIDFController turretController =  new PIDFController(new PIDFCoefficients(0.05 , 0, 0.002, 0));
        DcMotorEx shooterMotorLeft = hardwareMap.get(DcMotorEx.class, "motor_sl");
        shooterMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotorEx shooterMotorRight = hardwareMap.get(DcMotorEx.class, "motor_sr");
        shooterMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PIDFController shooterController = new PIDFController(new PIDFCoefficients(0.0075, 0, 0, 0.0005));

        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "motor_im");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        CRServo transferLeft = hardwareMap.get(CRServo.class, "servo_tl");
        transferLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        CRServo transferRight = hardwareMap.get(CRServo.class, "servo_tr");

        Servo leftPin = hardwareMap.get(Servo.class,"servo_lp");
        Servo centerPin = hardwareMap.get(Servo.class,"servo_cp");
        Servo rightPin = hardwareMap.get(Servo.class,"servo_rp");

        ColorSensor leftColorSensor = hardwareMap.get(ColorSensor.class, "sensor_cl");
        ColorSensor rightColorSensor = hardwareMap.get(ColorSensor.class, "sensor_cr");


        Servo leftLight = hardwareMap.get(Servo.class, "light_ll");
        Servo rightLight = hardwareMap.get(Servo.class, "light_rl");
        Servo centerLight = hardwareMap.get(Servo.class, "light_cl");

        Servo leftLift = hardwareMap.get(Servo.class, "servo_ll");
        Servo rightLift = hardwareMap.get(Servo.class, "servo_rl");
        rightLift.setDirection(Servo.Direction.REVERSE);

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limeLight");

        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(8, 5, Math.toRadians(90)));


        waitForStart();
        limelight.pipelineSwitch(0);
        limelight.start();
        while (!isStopRequested()){
            timer.start();
            leftLift.setPosition(liftPos);
            rightLift.setPosition(liftPos);
            if (turretTargetPosition != 0){
                turretController.setTargetPosition(turretTargetPosition);
                turretController.updatePosition(turret.getCurrentPosition());
                turret.setPower(turretController.run());

            }
            if (shooterTargetVelocity != 0){
                shooterController.updateFeedForwardInput(shooterTargetVelocity);
                shooterController.setTargetPosition(shooterTargetVelocity);
                shooterController.updatePosition(shooterMotorLeft.getVelocity());
                shooterMotorRight.setPower(shooterController.run());
                shooterMotorLeft.setPower(shooterController.run());
                double distance = follower.getPose().distanceFrom(goalPos);
                telemetry.addData("TargetVelocity", shooterTargetVelocity);
                telemetry.addData("CurrentVelocity", shooterMotorLeft.getVelocity());
                telemetry.addData("Power", shooterController.run());
                telemetry.addData("distance", distance);
                telemetry.update();
            }
            if (intakePower != 0){
                intakeMotor.setPower(intakePower);
                telemetry.addData("intakeCurrent", intakeMotor.getCurrent(CurrentUnit.AMPS));
                telemetry.update();


            }
            if (transferPower != 0){
                transferRight.setPower(transferPower);
                transferLeft.setPower(transferPower);

            }
            if (usePins){
                centerPin.setPosition(centerPinPosition);
                leftPin.setPosition(leftPinPositon);
                rightPin.setPosition(rightPinPosition);
            }
            if (turretAutoAim){
                double targetAngle;
                double turretAngle;
                boolean useVision;
                if (turret.getCurrentPosition() != 0) {
                    turretAngle = -360 / ((4096 * ((double) 70 / 20)) / turret.getCurrentPosition());
                } else {
                    turretAngle = 0;
                }



                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    double tx = result.getTx() ;
                    targetAngle = turretAngle - tx;
                    useVision = true;
                } else {

                    Pose robotPos = follower.getPose();
                    double gobleAngle = Math.atan2(goalPos.getY() - robotPos.getY(), goalPos.getX() - robotPos.getX());
                    double turretHeading = robotPos.getHeading() + Math.PI;
                    if (turretHeading > Math.PI) turretHeading -= 2 * Math.PI;
                    if (turretHeading < -Math.PI) turretHeading += 2 * Math.PI;
                    targetAngle = Math.toDegrees(gobleAngle - turretHeading);
                    if (targetAngle > 180) targetAngle -= 360;
                    useVision = false;
                }
                boolean atAngle = turretAngle > targetAngle - 1 && turretAngle < targetAngle + 1;

                if (Math.abs(targetAngle) < 125 && !atAngle) {
                    turretController.setTargetPosition(targetAngle);
                    turretController.updatePosition(turretAngle);
                    turret.setPower(turretController.run() );
                } else turret.setPower(0);
            }

            if (leftColorSensor.blue()<100 &&leftColorSensor.red()<100 && leftColorSensor.green()<100){

                leftLight.setPosition(0.28);


            }
            else if(leftColorSensor.green() > leftColorSensor.blue() && leftColorSensor.green() > leftColorSensor.red()){

                leftLight.setPosition(0.444);
            }
            else {

                leftLight.setPosition(0.722);

            }
            if (rightColorSensor.blue()<100 &&rightColorSensor.red()<100 && rightColorSensor.green()<100){

                rightLight.setPosition(0.28);

            }
            else if(rightColorSensor.green() > rightColorSensor.blue() && rightColorSensor.green() > rightColorSensor.red()){

                rightLight.setPosition(0.444);
            }
            else {

                rightLight.setPosition(0.722);
            }

            limelight.updateRobotOrientation(Math.toDegrees(follower.getHeading()));


            follower.update();
            timer.end();

            telemetry.update();




        }

    }
}
