package org.firstinspires.ftc.teamcode.prototyping;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class IntakeTest extends OpMode {

    private enum intake_state{
        In,
        Out,
        OnlyA,
        OnlyB,
        Off
    }

    final double Stop_speed = 0.0;

    double intake_speed = 700;

    double outtake_speed = -700;

    intake_state intakeState = intake_state.Off;

    DcMotorEx intake_a;
    DcMotorEx intake_b;

    @Override
    public void init(){

        DcMotorEx Intake_a = hardwareMap.get(DcMotorEx.class, "Intake_a");
        DcMotorEx Intake_b = hardwareMap.get(DcMotorEx.class, "Intake_b");

        Intake_a.setVelocity(Stop_speed);
        Intake_b.setVelocity(Stop_speed);
    }

    @Override
    public void init_loop() {

        // Letting the user know that we are waiting for the Intake Mode
        telemetry.addData("Choose Intake Mode: ","X - In, B - Out");

        // Checking User Input
        if (gamepad1.x) { // <-- This checks if x has been pressed
            intakeState = intake_state.In; // Assigns the Mode as In
        }
        if (gamepad1.b) { // <-- This checks if b has been pressed.
            intakeState = intake_state.Out; // Assigns the Mode as Out
        }

        // Letting the user know either what type of launcher they have chosen or letting them know that they have not chosen one yet
        telemetry.addData("Chosen Intake Mode", intakeState);
        telemetry.update();
    }

    @Override
    public void loop(){

        switch(intakeState) {
            case In:

                //changes intake motor B to Out
                if (gamepad1.left_bumper){
                    intakeState = intake_state.OnlyA;
                }

                //changes intake motor A to Out
                if (gamepad1.right_bumper){
                    intakeState = intake_state.OnlyB;
                }

                //changes intake to Off
                if (gamepad1.a){
                    intakeState = intake_state.Off;
                }

                //changes intake to Out
                if (gamepad1.b){
                    intakeState = intake_state.Out;
                }


                //Sets motors
                intake_a.setVelocity(intake_speed);
                intake_b.setVelocity(intake_speed);

            case Out:

                //changes intake motor A to in
                if (gamepad1.left_bumper){
                    intakeState = intake_state.OnlyA;
                }

                //changes intake motor B to in
                if (gamepad1.right_bumper){
                    intakeState = intake_state.OnlyB;
                }

                //changes intake to Off
                if (gamepad1.a){
                    intakeState = intake_state.Off;
                }

                //changes intake to In
                if (gamepad1.b){
                    intakeState = intake_state.In;
                }


                //Sets motors
                intake_a.setVelocity(outtake_speed);
                intake_b.setVelocity(outtake_speed);

            case OnlyA:

                //changes intake motor B to In
                if (gamepad1.left_bumper){
                    intakeState = intake_state.In;
                }

                //changes intake motor A to Out
                if (gamepad1.right_bumper){
                    intakeState = intake_state.Out;
                }

                //changes intake to Off
                if (gamepad1.a){
                    intakeState = intake_state.Off;
                }

                //changes intake to Only B In
                if (gamepad1.b){
                    intakeState = intake_state.OnlyB;
                }


                //Sets motors
                intake_a.setVelocity(intake_speed);
                intake_b.setVelocity(outtake_speed);

            case OnlyB:

                //changes intake motor B to Out
                if (gamepad1.left_bumper){
                    intakeState = intake_state.Out;
                }

                //changes intake motor A to In
                if (gamepad1.right_bumper){
                    intakeState = intake_state.In;
                }

                //changes intake to Off
                if (gamepad1.a){
                    intakeState = intake_state.Off;
                }

                //changes intake to Out
                if (gamepad1.b){
                    intakeState = intake_state.OnlyA;
                }


                //Sets motors
                intake_a.setVelocity(outtake_speed);
                intake_b.setVelocity(intake_speed);

            case Off:

                // Checking User Input
                if (gamepad1.x) { // <-- This checks if x has been pressed
                    intakeState = intake_state.In; // Assigns the Mode as In
                }
                if (gamepad1.b) { // <-- This checks if b has been pressed.
                    intakeState = intake_state.Out; // Assigns the Mode as Out
                }
                telemetry.addData("Choose Intake Mode: ","X - In, B - Out");
        }

        telemetry.addData("Mode:", intakeState);
        telemetry.addData("Intake Speed:", intake_speed);
        telemetry.addData("Outtake Speed:", outtake_speed);
        telemetry.update();
    }
}