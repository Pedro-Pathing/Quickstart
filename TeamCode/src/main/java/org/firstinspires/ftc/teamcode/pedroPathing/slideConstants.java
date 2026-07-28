package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class slideConstants {
    private DcMotor containerMotor;
    static final double TICKS_PER_REV = 537.6;
    static final double ARM_TICKS_PER_REV = 1425.1;
    static final int POSITION_RETRACTED = 0;
    static final int POSITION_MIDDLE = -(int)(TICKS_PER_REV * 1.1);
    static final int POSITION_MAX = -(int)(TICKS_PER_REV * 2.3);
    static final double SLIDE_POWER = 0.3;

    public slideConstants(HardwareMap hardwareMap) {
        containerMotor = hardwareMap.get(DcMotor.class, "containerMotor");
        containerMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        containerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        containerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        containerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        containerMotor.setPower(0.0);
    }

    public void start() {
        containerMotor.setTargetPosition(POSITION_RETRACTED);
        containerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        containerMotor.setPower(SLIDE_POWER);
    }

    public void setTarget(int ticks) {
        if (ticks > POSITION_RETRACTED) ticks = POSITION_RETRACTED;
        if (ticks < POSITION_MAX) ticks = POSITION_MAX;

        containerMotor.setTargetPosition(ticks);
        containerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        containerMotor.setPower(SLIDE_POWER);
    }

    public void extendToHigh() { setTarget(POSITION_MAX); }
    public void extendToMiddle() { setTarget(POSITION_MIDDLE); }
    public void extendToBottom() { setTarget(POSITION_RETRACTED); }

    public int getCurrentPosition() {
        return containerMotor.getCurrentPosition();
    }

    public void resetEncoder() {
        containerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        containerMotor.setTargetPosition(POSITION_RETRACTED);
        containerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        containerMotor.setPower(SLIDE_POWER);
    }
}
