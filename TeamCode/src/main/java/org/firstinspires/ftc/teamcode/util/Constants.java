package org.firstinspires.ftc.teamcode.util;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.hardware.subsystems.CameraSubsystem;

@Configurable
public class Constants {

    /* -------------------------------------------- DRIVE CONSTANTS -------------------------------------------- */


    public static double pathEndXTolerance = 1;
    public static double pathEndYTolerance = 1;
    public static double pathEndHeadingTolerance = Math.toRadians(2);

    public static boolean robotCentric = true;

    /* -------------------------------------------- CAMERA CONSTANTS -------------------------------------------- */
    //Pipeline: 0
    //Res: 1280X960 40FPS
    //Exposure: 252
    // Black Level Offset: 0
    // Sensor Gain: 15
    // Marker Size 101.6
    // Detector Downscale: 4
    // Quality Threshold: 2
    // Sort Mode: Largest

    /* -------------------------------------------- INTAKE CONSTANTS -------------------------------------------- */

    public static double intakeInPower = -0.9;
    public static double intakeOutPower = 1;

    /* -------------------------------------------- SHOOT CONSTANTS -------------------------------------------- */

    public static double shootPower = 0.94;
    public static double readyPower = -1.0;
    public static double reverseStopPower = 1;

    public static double kP = 0.0004; // to make response faser
    public static double kI = 0.00005; // for undershoot
    public static double kD = 0.0; // dont change
    public static double kF = 32767 / ((28 * 6000) / 60.0); // default must tune


    /* -------------------------------------------- TURRET CONSTANTS -------------------------------------------- */
    // Controller helper params
    public static double deadbandDeg = 0.30;
    public static double errAlpha = 0.35;

    // Safety rails
    public static double maxIntegral = 30.0;   // deg·s (anti-windup)
    public static double maxDeriv = 320.0;  // deg/s (D clamp)

    // CR servo output limits
    public static double maxPower = 1.0;
    public static double kS = 0.0;      //set to 0.03 is something is still messing up

    // PID gains mapping error->power (tune these)
    public static double kP_v = 0.020;
    public static double kI_v = 0.000;
    public static double kD_v = 0.0010;
}

