package org.firstinspires.ftc.teamcode.fwheel;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="kV", group="TeleOp")
public class kVTuner extends OpMode
{
    Flywheel flywheel = new Flywheel();

    public static double kV = 0.01;

    public double kS = 0.043488;
    public double goalRPM = 2700;

    double[] increments = {0.000001, 0.00001, 0.0001, 0.001, 0.01};

    int incrementIdx = 4;

    @Override
    public void init()
    {
        flywheel.init(hardwareMap);
    }

    @Override
    public void loop()
    {
        if(gamepad1.dpadRightWasPressed() && incrementIdx < 4)
        {
            incrementIdx++;
        }
        else if (gamepad1.dpadLeftWasPressed() && incrementIdx > 0)
        {
            incrementIdx--;
        }

        if(gamepad1.a)
        {
            goalRPM = 2700;
        }
        else if(gamepad1.b)
        {
            goalRPM = 1500; //temp value
        }

        double currentStep = increments[incrementIdx];

        if(gamepad1.dpadUpWasPressed())
        {
            kV +=currentStep;
        }
        if(gamepad1.dpadDownWasPressed())
        {
            kV -=currentStep;
        }

        double power = (kV * goalRPM) + kS;

        flywheel.setPower(power);

        telemetry.addData("Step", "%.6f", currentStep);
        telemetry.addData("kV","%.6f", kV);
        telemetry.addData("RPM",flywheel.getRPM());
        telemetry.addData("Ticks per Sec", flywheel.getTicksPerSec());
    }
}
