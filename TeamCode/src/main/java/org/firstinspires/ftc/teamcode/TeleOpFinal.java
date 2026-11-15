package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;

@TeleOp(name = "Drive", group = "TeleOp")
public class TeleOpFinal extends LinearOpMode
{
    boolean stopperEnable = true;


    private static final double SLOW_MODE_SCALE = 0.35;

    // Match timing (seconds). Teleop is 2:00; flowers open with 60 s left.
    private static final double TELEOP_LENGTH_S = 120.0;
    private static final double FLOWERS_OPEN_REMAINING_S = 60.0;
    private static final double FINAL_WARNING_REMAINING_S = 30.0;
    private boolean flowersWarned = false;
    private boolean finalWarned = false;

    private DcMotor shooter_driver;

    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeft, backLeft, frontRight, backRight;
    private Intake intake;
    private DcMotor shooter_driver_two;

    //private DcMotor intake_right;
    private DcMotor intake_left;

    private Servo stopper;
//
    @Override
    public void runOpMode() {
        frontLeft  = hardwareMap.get(DcMotor.class, HardwareNames.FRONT_LEFT);
        backLeft   = hardwareMap.get(DcMotor.class, HardwareNames.BACK_LEFT);
        frontRight = hardwareMap.get(DcMotor.class, HardwareNames.FRONT_RIGHT);
        backRight  = hardwareMap.get(DcMotor.class, HardwareNames.BACK_RIGHT);
        shooter_driver = hardwareMap.get(DcMotor.class, HardwareNames.SHOOTER_DRIVER);
        shooter_driver_two = hardwareMap.get(DcMotor.class, "shooter_two");
        intake_left = hardwareMap.get(DcMotor.class, HardwareNames.INTAKE_LEFT);
        stopper = hardwareMap.get(Servo.class, HardwareNames.STOPPER);
        ((ServoImplEx) stopper).setPwmRange(new PwmControl.PwmRange(500, 2500));
       // intake_right = hardwareMap.get(DcMotor.class, HardwareNames.INTAKE_RIGHT);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        stopper.setPosition(0.45);
        shooter_driver.setDirection(DcMotorSimple.Direction.FORWARD);
        for (DcMotor m : new DcMotor[]{frontLeft, backLeft, frontRight, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        intake = new Intake(hardwareMap);


        telemetry.addData("Status", "Initialized - press START");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double axial   = Math.pow(-gamepad1.left_stick_y, 3);
            double lateral =  Math.pow(gamepad1.left_stick_x, 3);
            double yaw     =  Math.pow(gamepad1.right_stick_x, 3);

            double flPower = axial + lateral + yaw;
            double frPower = axial - lateral - yaw;
            double blPower = axial - lateral + yaw;
            double brPower = axial + lateral - yaw;

            double max = Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)),
                                  Math.max(Math.abs(blPower), Math.abs(brPower)));
            if (max > 1.0) {
                flPower /= max;
                frPower /= max;
                blPower /= max;
                brPower /= max;
            }

            boolean slow = gamepad1.right_bumper;
            double scale = slow ? SLOW_MODE_SCALE : 1.0;

            frontLeft.setPower(flPower * scale);
            frontRight.setPower(frPower * scale);
            backLeft.setPower(blPower * scale);
            backRight.setPower(brPower * scale);

            double remaining = TELEOP_LENGTH_S - runtime.seconds();
            if (!flowersWarned && remaining <= FLOWERS_OPEN_REMAINING_S) {
                flowersWarned = true;
                gamepad1.rumbleBlips(2);
            }
            if (!finalWarned && remaining <= FINAL_WARNING_REMAINING_S) {
                finalWarned = true;
                gamepad1.rumbleBlips(3);
            }
            //intake
            if (gamepad1.a) {
                intake.out();
                //intake.out();


            } else if (gamepad1.b) {
                intake.in();
                //intake.in();
            } else {
                intake.stop();
            }
            //flywheel
            if(gamepad1.x){
                shooter_driver.setPower(0.45);
                shooter_driver_two.setPower(0.7);
                //stopper.setPosition(0.47);

            }
            if(gamepad1.x && shooter_driver.getPower() > 0){
                shooter_driver.setPower(0);
                shooter_driver_two.setPower(0);
                //stopper.setPosition(0);
            }
            if(gamepad1.dpad_left){
                runtime.reset();


//                if(runtime.seconds() > 5){
//                    stopper.setPosition(0);
//
//                }
                stopper.setPosition(0.35);

            }
            else{
                runtime.reset();


                if(runtime.milliseconds() > 15) {
                    stopper.setPosition(0.45);
                }
                //stopperEnable = true;
            }


            telemetry.addData("Time left", "%.0f s", Math.max(0, remaining));
            telemetry.addData("Slow mode", slow ? "ON" : "off");
            telemetry.addData("Intake power", "%.2f", intake.getPower());
            telemetry.addData("Front L/R", "%.2f, %.2f", flPower * scale, frPower * scale);
            telemetry.addData("Back  L/R", "%.2f, %.2f", blPower * scale, brPower * scale);
            telemetry.addData("Stopper angle:" ,"%.2f", stopper.getPosition());
            telemetry.update();
        }
    }
}
