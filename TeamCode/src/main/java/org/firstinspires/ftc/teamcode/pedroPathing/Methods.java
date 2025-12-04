package org.firstinspires.ftc.teamcode.pedroPathing;
import com.arcrobotics.ftclib.controller.PIDFController;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Methods {
    private boolean firstLoop = true;
    private double lastPos;
    private double lastTime;

    public void FlyWheel_PID(DcMotorEx motor, double targetVelocity) {
        double kF, kD, kP,kI;
        PIDFController controller;
        kI = Values.flywheel_Values.fi;
        kD = Values.flywheel_Values.fd;
        kP = Values.flywheel_Values.fp;
        kF = Values.flywheel_Values.ff;
        controller = Values.flywheel_Values.flywheelPIDController;
        if (firstLoop) {
            lastPos = motor.getCurrentPosition();
            lastTime = System.nanoTime() / 1e9;
            firstLoop = false;
            return;
        }
        double currentPos = motor.getCurrentPosition();
        double currentTime = System.nanoTime() / 1e9;

        double dt = currentTime - lastTime;
        double dp = currentPos - lastPos;

        double measuredVelocity = dp / dt;

        lastPos = currentPos;
        lastTime = currentTime;

        if (targetVelocity == 0) {
            motor.setPower(0);
            return;
        }

        controller.setPIDF(kP, kI, kD, kF);
        double power = controller.calculate(measuredVelocity, targetVelocity);
        motor.setPower(power);
    }

    public void resetPID() {
        firstLoop = true;
    }

    public void Relocalize(){
        return;
    }
    public void Transfer(){
    }
    public void AutoAim(){
    }

}
/// intake: PID,
/// Hood: PID,
/// flywheel: PID
/// transfer: PID, autoaim turret
/// ll: relocalization