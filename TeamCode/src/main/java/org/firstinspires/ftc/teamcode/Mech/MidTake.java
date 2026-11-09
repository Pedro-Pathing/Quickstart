package org.firstinspires.ftc.teamcode.Mech;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MidTake {

    private DcMotorEx midtake;
    private double lastTuneP = -1, lastTuneI = -1, lastTuneD = -1, lastTuneF = -1;

    public void init(HardwareMap hardwareMap){

        midtake = hardwareMap.get(DcMotorEx.class, "midTake");
        midtake.setDirection(DcMotorEx.Direction.FORWARD);
        midtake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        midtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        midtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void spin(boolean RBp1, boolean Yp2){

        if (RBp1 || Yp2){
            midtake.setVelocityPIDFCoefficients(60, 0,0, 16.67);
            midtake.setVelocity(1578);
        }
        else{
            midtake.setVelocity(0);
        }
    }

    public void Tune(double P, double I, double D, double F, double rpm){
        if (P != lastTuneP || I != lastTuneI || D != lastTuneD || F != lastTuneF) {
            midtake.setVelocityPIDFCoefficients(P, I, D, F);
            lastTuneP = P;
            lastTuneI = I;
            lastTuneD = D;
            lastTuneF = F;
        }
        midtake.setVelocity(rpm);
    }

    public double getVelocity(){ return midtake.getVelocity();}

}
