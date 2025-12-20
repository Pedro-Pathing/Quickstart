package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {
    public Limelight3A ll;
    public DcMotorEx intake, transfer, flywheel1, flywheel2;
    public Servo led;
    public Servo hood1, hood2, turret1, turret2;

    public HardwareMap map;



    public Hardware(HardwareMap hardwareMap) {
        this.map = hardwareMap;
    }
    public void hardware (){
        ll = map.get(Limelight3A.class, "ll");
        intake = map.get(DcMotorEx.class, "intake");
        transfer = map.get(DcMotorEx.class, "transfer");
        flywheel1 = map.get(DcMotorEx.class, "shooter1");
        flywheel2 = map.get(DcMotorEx.class, "shooter2");
        hood1 = map.get(Servo.class, "hood1");
        hood2 = map.get(Servo.class, "hood2");
        turret1 = map.get(Servo.class, "turret1");
        turret2 = map.get(Servo.class, "turret2");
        led = map.get(Servo.class, "led");
        ll.pipelineSwitch(1);
        flywheel1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel1.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel1.setPower(0);
        flywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setDirection(DcMotorEx.Direction.FORWARD);
        flywheel2.setPower(0);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setPower(0);
        transfer.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setDirection(DcMotorEx.Direction.REVERSE);
        transfer.setPower(0);
    }
}
