package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class Intake {
    private final Devices.DcMotorExClass intake = new Devices.DcMotorExClass();
    private final Devices.CRServoClass belt = new Devices.CRServoClass();

    public enum Mode {
        INTAKING,
        SHOOTING,
        REVERSING,
        OFF
    }

    public void init(OpMode opmode){
        intake.init(opmode, "intake");
        belt.init(opmode, "belt");

        intake.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        belt.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setMode(Mode direction){
        if (direction == Mode.INTAKING){
            intake.setPower(1);
            belt.setPower(1);
        }else if (direction == Mode.SHOOTING){
            intake.setPower(0);
            belt.setPower(1);
        }else if (direction == Mode.REVERSING){
            intake.setPower(-1);
            belt.setPower(-1);
        }else{
            intake.setPower(0);
            belt.setPower(0);
        }
    }

    public Mode getMode(){
        double intakePower = intake.getPower();
        double beltPower = belt.getPower();
        if (intakePower == 1 && beltPower == 1){
            return Mode.INTAKING;
        }else if (intakePower == 0 && beltPower == 1){
            return Mode.SHOOTING;
        }else if (intakePower == -1 && beltPower == -1){
            return Mode.REVERSING;
        }else{
            return Mode.OFF;
        }
    }
}
