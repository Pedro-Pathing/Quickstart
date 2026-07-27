package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.lynx.LynxServoController;

/* JADX INFO: loaded from: classes7.dex */
public class SimplePIDController {
    private double integralSum;
    private double kD;
    private double kI;
    private double kP;
    private double previousError;
    private boolean firstUpdate = true;
    private double integralMin = Double.NEGATIVE_INFINITY;
    private double integralMax = Double.POSITIVE_INFINITY;

    public SimplePIDController(double kP, double kI, double kD) {
        setCoefficients(kP, kI, kD);
    }

    public void setCoefficients(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setIntegralBounds(double integralMin, double integralMax) {
        this.integralMin = integralMin;
        this.integralMax = integralMax;
    }

    public double calculate(double error, double dtSeconds) {
        if (dtSeconds <= LynxServoController.apiPositionFirst) {
            dtSeconds = 0.001d;
        }
        this.integralSum += error * dtSeconds;
        this.integralSum = Math.max(this.integralMin, Math.min(this.integralSum, this.integralMax));
        double derivative = LynxServoController.apiPositionFirst;
        if (!this.firstUpdate) {
            derivative = (error - this.previousError) / dtSeconds;
        } else {
            this.firstUpdate = false;
        }
        this.previousError = error;
        return (this.kP * error) + (this.kI * this.integralSum) + (this.kD * derivative);
    }

    public void reset() {
        this.integralSum = LynxServoController.apiPositionFirst;
        this.previousError = LynxServoController.apiPositionFirst;
        this.firstUpdate = true;
    }

    public double getIntegralSum() {
        return this.integralSum;
    }
}
