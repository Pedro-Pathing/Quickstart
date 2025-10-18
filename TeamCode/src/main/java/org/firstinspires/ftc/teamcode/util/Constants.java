package org.firstinspires.ftc.teamcode.util;
import org.firstinspires.ftc.teamcode.hardware.subsystems.CameraSubsystem;

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

    public static final double intakeInPower = 1;
    public static final double intakeOutPower = -1;

    /* -------------------------------------------- SHOOT CONSTANTS -------------------------------------------- */

    public static final double shootPower = 0.7;
    public static final double readyPower = -1.0;
}
