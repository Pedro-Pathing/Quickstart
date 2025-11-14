package org.firstinspires.ftc.teamcode.pedroPathing;



import static org.firstinspires.ftc.teamcode.pedroPathing.Robot.OnRPM;
import static org.firstinspires.ftc.teamcode.pedroPathing.Robot.intakePower;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Deque;
import java.util.LinkedList;

@TeleOp
@Configurable
public class RobotTele extends OpMode{

    Gamepad lastGamepad1 = new Gamepad(), lastGamepad2 = new Gamepad();
    Deque<Gamepad> gamepad1History = new LinkedList<>(), gamepad2History = new LinkedList<>();

    Robot bot = null;

    boolean bumped = false;
    boolean notBumped = true;
    boolean spinnerOn = false;
    boolean intakeOn = false;

    @Override
    public void init() {
        bot = new Robot(hardwareMap);

    }

    @Override
    public void loop() {
        if (gamepad2.start || gamepad1.start) return;






        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x * 1.1;
        double rx = -gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        if (gamepad1.right_trigger != 0) {
            double power = (1 - gamepad1.right_trigger);
            bot.leftFront.setPower(Math.max(power, 0.45) * frontLeftPower);
            bot.leftBack.setPower(Math.max(power, 0.45) * backLeftPower);
            bot.rightFront.setPower(Math.max(power, 0.45)  * frontRightPower);
            bot.rightBack.setPower(Math.max(power, 0.45) * backRightPower);
        } else {
            bot.leftFront.setPower(frontLeftPower);
            bot.leftBack.setPower(backLeftPower);
            bot.rightFront.setPower(frontRightPower);
            bot.rightBack.setPower(backRightPower);
        }



        if (gamepad2.b && !intakeOn) {
            bot.intake.setPower(intakePower);
            intakeOn = true;
        } else if (gamepad2.b && intakeOn) {
            bot.intake.setPower(0);
            intakeOn = false;
        }

        /*
        if (gamepad2.a && !spinnerOn) {
            bot.launcher1.setPower(1);
            bot.launcher2.setPower(1);
            spinnerOn = true;
        } else if (gamepad2.a && spinnerOn) {
            bot.launcher1.setPower(0);
            bot.launcher2.setPower(0);
            spinnerOn = false;
        }

         */


        if (gamepad2.a && !spinnerOn) {
            bot.RPMshooter(OnRPM);
            spinnerOn = true;
        } else if (gamepad2.a && spinnerOn) {
            bot.RPMshooter(0);
            spinnerOn = false;
        }



        if (gamepad2.x && !bumped && notBumped) {
            bot.bumper.setPosition(bot.bumperUp);
            bumped = true;
            notBumped = false;
        }
        else if (gamepad2.x && bumped && !notBumped) {
            bot.bumper.setPosition(bot.bumperRest);
            bumped = false;
            notBumped = true;
        }

        double power = gamepad1.left_trigger;
        bot.spindexer.setPower(power/5);



        gamepad1History.add(gamepad1);
        gamepad2History.add(gamepad2);

        if (gamepad1History.size() > 100) {
            gamepad1History.removeLast();
            gamepad2History.removeLast();
        }

        lastGamepad1.copy(gamepad1);
        lastGamepad2.copy(gamepad2);

    }
}
