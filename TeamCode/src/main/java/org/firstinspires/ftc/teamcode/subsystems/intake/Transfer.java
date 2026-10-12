package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.util.Utility;

public class Transfer implements Subsystem {

    //---------------- Hardware ----------------
    private final DcMotorEx transfer;
    private final Servo blocker;
    //TODO figure out which distance we are using and implement them

    //---------------- Software ----------------
    private Utility util;

    private double transferPow = 0.0;
    private double blockerPos = 0.0;
    private boolean blockerChanged = false;

    private final double blockerStart = 0.5;
    //TODO find blocker limits
    private final double blockerMin = 0.0;
    private final double blockerMax = 1.0;

    //---------------- Constructor ----------------
    public Transfer(HardwareMap map) {
        transfer = map.get(DcMotorEx.class, "transfer");
        blocker = map.get(Servo.class, "blocker");

    }

    //---------------- Core Methods ----------------
    private void setTransfer(double pow){
        transfer.setPower(pow);
    }

    private void setBlocker(double pos){
        blocker.setPosition(util.clamp(pos, blockerMin, blockerMax));
    }

    //---------------- Methods ----------------
    public void setTransferPow(double pow){
        transferPow = pow;
    }

    public void setBlockerPos(double pos){
        blockerChanged = true;
        blockerPos = pos;
    }

    //---------------- Interface Methods ----------------
    @Override
    public void init(){
        setBlockerPos(blockerStart);
    }

    @Override
    public void update(){
        setTransfer(transferPow);

        if (blockerChanged){
            setBlocker(blockerPos);
            blockerChanged = false;
        }
    }
}
