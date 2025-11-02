package org.firstinspires.ftc.teamcode.util;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.hardware.subsystems.CameraSubsystem;

@Configurable
public class Constants {

    /* -------------------------------------------- DRIVE CONSTANTS -------------------------------------------- */


    public static double pathEndXTolerance = 1;
    public static double pathEndYTolerance = 1;
    public static double pathEndHeadingTolerance = Math.toRadians(2);

    public static boolean robotCentric = false;

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

    public static final double intakeInPower = -1;
    public static final double intakeOutPower = 1;

    /* -------------------------------------------- SHOOT CONSTANTS -------------------------------------------- */

    public static final double shootPower = 1;
    public static final double readyPower = -1.0;
    public static final double reverseStopPower = 1;


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

