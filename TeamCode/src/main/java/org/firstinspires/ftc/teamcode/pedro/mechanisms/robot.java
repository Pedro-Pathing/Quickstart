package org.firstinspires.ftc.teamcode.pedro.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

// Create robot motor variables and their getters
public class robot {
    private DcMotorEx FL;
    public DcMotorEx getFL() {
        return FL;
    }
    private DcMotorEx FR;
    public DcMotorEx getFR() {
        return FR;
    }
    private DcMotorEx BL;
    public DcMotorEx getBL() {
        return BL;
    }
    private DcMotorEx BR;
    public DcMotorEx getBR() {
        return BR;
    }
    private DcMotorEx turret;
    public DcMotorEx getTurret() {
        return turret;
    }
    private DcMotorEx leftShoot;
    public DcMotorEx getLeftShoot() {
        return leftShoot;
    }
    private DcMotorEx rightShoot;
    public DcMotorEx getRightShoot() {
        return rightShoot;
    }
    private DcMotorEx intake;
    public DcMotorEx getIntake() {
        return intake;
    }
    private CRServo rightLaunch;
    public CRServo getRightLaunch() {
        return rightLaunch;
    }
    private CRServo leftLaunch;
    public CRServo getLeftLaunch() {
        return leftLaunch;
    }

    // Connect each motor variable to the motor in the configuration
    public void init(HardwareMap hwMap) {
        FL = hwMap.get(DcMotorEx.class, "FL");
        FR = hwMap.get(DcMotorEx.class, "FR");
        BL = hwMap.get(DcMotorEx.class, "BL");
        BR = hwMap.get(DcMotorEx.class, "BR");
        turret = hwMap.get(DcMotorEx.class, "turret");
        leftShoot = hwMap.get(DcMotorEx.class, "leftShoot");
        rightShoot = hwMap.get(DcMotorEx.class, "rightShoot");
        intake = hwMap.get(DcMotorEx.class, "intake");
        rightLaunch = hwMap.get(CRServo.class, "rightLaunch");
        leftLaunch = hwMap.get(CRServo.class, "leftLaunch");

        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        turret.setDirection(DcMotor.Direction.REVERSE);
        leftShoot.setDirection(DcMotor.Direction.REVERSE);
        leftLaunch.setDirection(CRServo.Direction.REVERSE);

        // Set each motor's mode
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftShoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftShoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(DcMotorEx motor, double power) {
        motor.setPower(power);
    }
    public void runToPosition(DcMotorEx motor, int position, double speed, int range) {
        motor.setTargetPositionTolerance(range);
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(speed);
    }
}
