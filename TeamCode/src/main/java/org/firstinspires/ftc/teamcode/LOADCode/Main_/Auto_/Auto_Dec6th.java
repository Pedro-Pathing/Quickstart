package org.firstinspires.ftc.teamcode.LOADCode.Main_.Auto_; // make sure this aligns with class location

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Auto_Dec6th", group = "TestAuto", preselectTeleOp="Teleop_Main_")
public class Auto_Dec6th extends OpMode {

    DcMotorEx FL;
    DcMotorEx FR;
    DcMotorEx BL;
    DcMotorEx BR;

    /**
     * Copied over from LinearOpMode.
     * @param milliseconds The number of milliseconds to sleep.
     */
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {

    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        sleep(25000);
        FL.setPower(0.3);
        FR.setPower(0.3);
        BL.setPower(0.3);
        BR.setPower(0.3);
        sleep(1700);
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {

    }
}