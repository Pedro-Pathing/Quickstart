package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ColorSensorBench;

@TeleOp(name="colortest", group="FTC")
public class ColorTester extends OpMode {
    ColorSensorBench bench = new ColorSensorBench();

    @Override
    public void init (){
        bench.init(hardwareMap);
    }

    @Override
    public void loop() {
        ColorSensorBench.DetectedColor detectedColor = bench.getDetectedColor(telemetry);
        telemetry.addData("color Detected", detectedColor);
        telemetry.update();
    }
}


//"видишь зеленый артефакт делаешь первый rgb зеленый, видишь феолетовый делаешь первый rgb феолетовым.
//"так же с отстальными двумя"
