package org.firstinspires.ftc.teamcode.fwheel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Flywheel
{
    private DcMotorEx m1;

    private double encoderCPM = 384.5;

    private double gearRatio = 1;

    private double kV,kS,kP;

    public void init(HardwareMap hwMap)
    {
        m1 = hwMap.get(DcMotorEx.class, "motor");

        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double power)
    {
        m1.setPower(power);
    }

    public double getTicksPerSec()
    {
        return m1.getVelocity();
    }

    public double getRPM()
    {
        return ((getTicksPerSec()/encoderCPM)*60)/gearRatio;
    }


}
