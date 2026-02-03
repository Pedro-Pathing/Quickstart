package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="yeva", group="FTC")
public class yeva extends OpMode {

    //    private CRServo l1, l2;
    private DcMotor left1, left2, right1, right2;
    private DcMotor intake;
    private DcMotorEx shut1,shut2;
    private Servo pod;

    String last;

    private double liftPower = 1.0;
    private boolean lastYButtonState = false;

    private boolean lastXButtonState = false;

    // Состояние для toggle-логики far shooter
    private boolean farShooterActive = false;

    // PID параметры для FarShooter
    private static final double FAR_SHOOTER_P = 25;  // Вставьте значение P для FarShooter
    private static final double FAR_SHOOTER_F = 11.2;  // Вставьте значение F для FarShooter

    // Целевая скорость для FarShooter (в тиках в секунду)
    private static final double FAR_SHOOTER_VELOCITY = 1400;


    @Override
    public void init() {
        last = "";


        left1 = hardwareMap.get(DcMotor.class, "left1");
        left2 = hardwareMap.get(DcMotor.class, "left2");
        right1 = hardwareMap.get(DcMotor.class, "right2");
        right2 = hardwareMap.get(DcMotor.class, "right1");

        intake = hardwareMap.get(DcMotor.class, "intake");
        shut1 = hardwareMap.get(DcMotorEx.class, "shut1");
        shut2 = hardwareMap.get(DcMotorEx.class, "shut2");




        pod = hardwareMap.get(Servo.class, "pod");


        // Настройка моторов дальнего шута с PIDF
        shut1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shut2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Настройка направления
        shut1.setDirection(DcMotorSimple.Direction.FORWARD);
        shut2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Применение PIDF коэффициентов к обоим моторам (по умолчанию для FarShooter)
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(FAR_SHOOTER_P, 0, 0, FAR_SHOOTER_F);
        shut1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shut2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);


    }


    @Override
    public void loop() {
        double y = -gamepad1.right_stick_x;
        double x = -gamepad1.left_stick_x;
        double rotation = gamepad1.left_stick_y;

        double leftFrontPower = y + x + rotation;
        double leftBackPower = y - x + rotation;
        double rightFrontPower = y - x - rotation;
        double rightBackPower = y + x - rotation;

        left1.setPower(Math.max(-1, Math.min(1, leftFrontPower)));
        left2.setPower(Math.max(-1, Math.min(1, leftBackPower)));
        right1.setPower(Math.max(-1, Math.min(1, rightFrontPower)));
        right2.setPower(Math.max(-1, Math.min(1, rightBackPower)));


        // Управление intake на gamepad1 и gamepad2 (те же кнопки)
        if (gamepad1.left_trigger > 0 || gamepad2.right_bumper) {
            intake.setPower(-0.67);

        } else if (gamepad1.right_trigger > 0 || gamepad2.right_trigger > 0) {
            intake.setPower(0.67);

        } else {
            intake.setPower(0.0);

        }

        // Toggle-логика для FarShooter (кнопка Y на gamepad2)
        boolean currentYButtonState = gamepad2.y;
        if (currentYButtonState && !lastYButtonState) {
            farShooterActive = !farShooterActive;
            if (farShooterActive) {
                // Применяем PID параметры для FarShooter
                PIDFCoefficients pidfCoefficients = new PIDFCoefficients(FAR_SHOOTER_P, 0, 0, FAR_SHOOTER_F);
                shut1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
                shut2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            }
        }
        lastYButtonState = currentYButtonState;

        // Управление моторами far shooter в зависимости от состояния
        if (farShooterActive) {
            shut1.setVelocity(FAR_SHOOTER_VELOCITY);
            shut2.setVelocity(FAR_SHOOTER_VELOCITY);
        } else {
            shut1.setVelocity(0);
            shut2.setVelocity(0);
        }

//        if (gamepad2.a) {
//            pod.setPosition(0.4);
//
//        } else {
//            pod.setPosition(0.15);
//
//        }









    }








}
