package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * Copied from the SDK sample ConceptScanServo.
 *
 * This OpMode scans the nectar and pollen gate servos back and forward together until Stop is pressed.
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * Hardware config names: "nectar", "pollen"
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 */
@TeleOp(name = "Scan Servo Test", group = "Test")
@Disabled
public class ScanServoTestTeleOp extends LinearOpMode {

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    Servo   nectarServo;
    Servo   pollenServo;
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;


    @Override
    public void runOpMode() {

        nectarServo = hardwareMap.get(Servo.class, "nectar");
        pollenServo = hardwareMap.get(Servo.class, "pollen");

        // Wait for the start button
        telemetry.addData("nectar servo", nectarServo.getPosition());
        telemetry.addData(">", "Press Start to scan nectarServo and pollenServo." );
        telemetry.update();
        waitForStart();


        // Scan servos till stop pressed.
        while(opModeIsActive()){

            // slew the servos, according to the rampUp (direction) variable.
            if (rampUp) {
                // Keep stepping up until we hit the max value.
                position += INCREMENT ;
                if (position >= MAX_POS ) {
                    position = MAX_POS;
                    rampUp = !rampUp;   // Switch ramp direction
                }
            }
            else {
                // Keep stepping down until we hit the min value.
                position -= INCREMENT ;
                if (position <= MIN_POS ) {
                    position = MIN_POS;
                    rampUp = !rampUp;  // Switch ramp direction
                }
            }

            // Set the servos to the new position
            nectarServo.setPosition(position);
            pollenServo.setPosition(position);

            // Display the commanded value and what each servo reports back
            telemetry.addData("Servo Position", "%5.2f", position);
            telemetry.addData("nectarServo", "%5.2f (port %d)", nectarServo.getPosition(), nectarServo.getPortNumber());
            telemetry.addData("pollenServo", "%5.2f (port %d)", pollenServo.getPosition(), pollenServo.getPortNumber());
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            sleep(CYCLE_MS);
            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
