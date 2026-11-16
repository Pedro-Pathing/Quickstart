package org.firstinspires.ftc.teamcode.fwheel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Flywheel
{
    private DcMotorEx shooter_driver;

    private double encoderCPM = 384.5;

    private double gearRatio = 1;

    private double kV,kS= 0.043488,kP;

    public void init(HardwareMap hwMap)
    {
        shooter_driver = hwMap.get(DcMotorEx.class, "shooter_driver");

        shooter_driver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double power)
    {
        shooter_driver.setPower(power);
    }

    public void stopMotors()
    {
        shooter_driver.setPower(0);
    }

    public void setMotorRPM(double targetRPM)
    {
        //call this every loop
        double error = targetRPM - getRPM();
        double ff = (kV * targetRPM) + kS;
        double fb = error * kP;
        double power = ff+fb;

        setPower(power);
    }

    public double getTicksPerSec()
    {
        return shooter_driver.getVelocity();
    }

    public double getRPM()
    {
        return ((getTicksPerSec()/encoderCPM)*60)/gearRatio;
    }


}
