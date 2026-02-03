package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSensorBench {
    NormalizedColorSensor colorSensor;
    public enum DetectedColor {
        PURPLE,
        GREEN,
        UNKNOWN
    }
    
    public void init(HardwareMap hwMap) {
        colorSensor = hwMap.get(NormalizedColorSensor.class, "colorsensor");
    colorSensor.setGain(8);

    }
    
    public DetectedColor getDetectedColor(Telemetry telemetry) {
        if (colorSensor == null) {
            telemetry.addData("Error", "Color sensor not initialized");
            return DetectedColor.UNKNOWN;
        }
        
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float normRed, normGreen, normBlue;
        
        // Avoid division by zero
        if (colors.alpha > 0) {
            normRed = colors.red / colors.alpha;
            normGreen = colors.green / colors.alpha;
            normBlue = colors.blue / colors.alpha;
        } else {
            normRed = colors.red;
            normGreen = colors.green;
            normBlue = colors.blue;
        }

        telemetry.addData("red", normRed);
        telemetry.addData("green", normGreen);
        telemetry.addData("blue", normBlue);
        
        // Calculate total intensity to check if we have enough light
        float totalIntensity = normRed + normGreen + normBlue;
        if (totalIntensity < 0.1) {
            telemetry.addData("Warning", "Low light intensity");
            return DetectedColor.UNKNOWN;
        }
        
        // Find dominant color component
        float maxComponent = Math.max(Math.max(normRed, normGreen), normBlue);
        float minComponent = Math.min(Math.min(normRed, normGreen), normBlue);
        
        // Calculate color ratios
        float redRatio = normRed / totalIntensity;
        float greenRatio = normGreen / totalIntensity;
        float blueRatio = normBlue / totalIntensity;
        
        telemetry.addData("red ratio", redRatio);
        telemetry.addData("green ratio", greenRatio);
        telemetry.addData("blue ratio", blueRatio);
        
        // GREEN detection logic:
        // Green typically has:
        // - Green component is dominant (highest)
        // - Green ratio > 0.35 (at least 35% of total intensity)
        // - Red and Blue are relatively low
        // - Green is significantly higher than red and blue
        boolean isGreen = (normGreen == maxComponent) && 
                         (greenRatio > 0.35) &&
                         (normGreen > normRed * 1.2) && 
                         (normGreen > normBlue * 1.2) &&
                         (normGreen > 0.3);
        
        // PURPLE detection logic:
        // Purple typically has:
        // - Blue component is dominant (highest or close to highest)
        // - Both red and blue components are significant
        // - Green component is lower than blue (most important)
        // - Red and blue together significantly exceed green
        // - Blue ratio is high (typically > 0.4 for purple)
        boolean isPurple = (normBlue > 0.2) &&                    // Blue must be significant
                          (normRed > 0.15) &&                    // Red must be significant
                          (normBlue >= normGreen) &&             // Blue is at least as high as green
                          (normBlue >= normRed * 0.6) &&         // Blue is at least 60% of red (or more)
                          ((normRed + normBlue) > normGreen * 1.2) &&  // Red+Blue together exceed green by at least 20%
                          (blueRatio > 0.35);                    // Blue should be at least 35% of total intensity
        
        if (isGreen) {
            telemetry.addData("Detected", "GREEN");
            return DetectedColor.GREEN;
        } else if (isPurple) {
            telemetry.addData("Detected", "PURPLE");
            return DetectedColor.PURPLE;
        } else {
            telemetry.addData("Detected", "UNKNOWN");
            return DetectedColor.UNKNOWN;
        }
    }

}
