package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Generic_.CRServoClass;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Generic_.DcMotorExClass;

public class Intake {
    private static OpMode opMode;

    private final DcMotorExClass intake = new DcMotorExClass();
    private final CRServoClass belt = new CRServoClass();

    public enum Mode {
        INTAKE,
        SHOOTING,
        REVERSE,
        OFF
    }

    public void init(OpMode opmode){
        opMode = opmode;
        intake.init(opMode, "intake");
        belt.init(opMode, "belt");

        intake.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setMode(Mode direction){
        if (direction == Mode.INTAKE){
            intake.setPower(1);
            belt.setPower(1);
        }else if (direction == Mode.SHOOTING){
            intake.setPower(0);
            belt.setPower(1);
        }else if (direction == Mode.REVERSE){
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
            return Mode.INTAKE;
        }else if (intakePower == 0 && beltPower == 1){
            return Mode.SHOOTING;
        }else if (intakePower == -1 && beltPower == -1){
            return Mode.REVERSE;
        }else{
            return Mode.OFF;
        }
    }
}
