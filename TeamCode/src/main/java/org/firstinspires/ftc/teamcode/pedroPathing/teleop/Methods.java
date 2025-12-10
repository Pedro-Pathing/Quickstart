package org.firstinspires.ftc.teamcode.pedroPathing.teleop;
import com.arcrobotics.ftclib.controller.PIDFController;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Methods {
    private boolean firstLoop = true;
    private double lastPos;
    private double lastTime;

    public double velocity_PID(DcMotorEx motor, double targetVelocity, String mode) {
        PIDFController controller;
        double kF, kD, kP,kI;

        switch (mode) {
            case "flywheel":
                kI = Values.flywheel_Values.fI;
                kD = Values.flywheel_Values.fD;
                kP = Values.flywheel_Values.fP;
                kF = Values.flywheel_Values.fF;
                controller = Values.flywheel_Values.flywheelPIDController;
                break;
            case "intake":
                kI = Values.intake_Values.iI;
                kD = Values.intake_Values.iD;
                kP = Values.intake_Values.iP;
                kF = Values.intake_Values.iK;
                controller = Values.intake_Values.intakePIDController;
                break;
            case "transfer":
                kI = Values.transfer_Values.trI;
                kD = Values.transfer_Values.trD;
                kP = Values.transfer_Values.trP;
                kF = Values.transfer_Values.trF;
                controller = Values.transfer_Values.transferPIDController;
                break;

            default:
                throw new IllegalArgumentException("Error: " + mode);
        }

        if (firstLoop) {
            lastPos = motor.getCurrentPosition();
            lastTime = System.nanoTime() / 1e9;
            firstLoop = false;
            return 0;
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
            return 0;
        }

        controller.setPIDF(kP, kI, kD, kF);
        double power = controller.calculate(measuredVelocity, targetVelocity);
        motor.setPower(power);
        return measuredVelocity;

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
/// intake: PID,velocity
/// Hood: PID,position
/// flywheel: PID velocity
/// transfer: PID?, autoaim turret
/// ll: relocalization