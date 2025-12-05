package org.firstinspires.ftc.teamcode.friends;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Class to provide a helpful abstraction layer for accessing the HardwareMap
 */

public class HardwareMap {

    /*
        -----------------------------------------------------------------------
        | FRM               | Front Right Wheel     | Control Hub Motor 0     |
        --------------------+-----------------------+--------------------------
        | BRM               | Back Right Wheel      | Control Hub Motor 1     |
        --------------------+-----------------------+--------------------------
        | BLM               | Back Left Wheel       | Control Hub Motor 2     |
        --------------------+-----------------------+--------------------------
        | FLM               | Front Left Wheel      | Control Hub Motor 3     |
        -----------------------------------------------------------------------
        | Intake            | Intake Motor          | Expansion Hub Motor 1   |
        -----------------------------------------------------------------------
     */

    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;
    public DcMotor intakeMotor;

    public HardwareMap(com.qualcomm.robotcore.hardware.HardwareMap hardwaremap) {

        frontRightMotor = hardwaremap.get(DcMotor.class, "FRM");
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor = hardwaremap.get(DcMotor.class, "FLM");
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor = hardwaremap.get(DcMotor.class, "BRM");
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor = hardwaremap.get(DcMotor.class, "BLM");
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeMotor = hardwaremap.get(DcMotor.class, "Intake");
    }
}
