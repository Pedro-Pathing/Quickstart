package org.firstinspires.ftc.teamcode.prototyping;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "launcherTest")
public class LauncherTest extends OpMode {

    // Creating the enum that will be used to see what code should run based on what launchers the user wants to test
    private enum LauncherType{
        SINGLE,
        DUAL
    }

    // Assigning important things that we will use
    final double STOP_SPEED = 0.0; // Making a stop speed for the launchers also makes the code easier to read
    double towards_goal_launcher_velocity = 100; // Making the initial velocity that the user will later change
    double away_from_goal_launcher_velocity = 100; // Making the initial velocity that the user will later change
    LauncherType launcher_type = LauncherType.SINGLE; // Creating the state machine for our Launcher Type

    DcMotorEx towards_goal_launcher;
    DcMotorEx away_from_goal_launcher;
    DcMotorEx running_motor;
    double running_motor_velocity;



    @Override
    public void init(){

        towards_goal_launcher = hardwareMap.get(DcMotorEx.class, "towards_goal_launcher"); // Creating the towards goal launcher
        away_from_goal_launcher = hardwareMap.get(DcMotorEx.class, "away_from_goal_launcher"); // Creating the away from goal launcher

        away_from_goal_launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        running_motor = towards_goal_launcher; // Setting the running motor for when the state is set to SINGLE
        running_motor_velocity = 100; // Setting the running motor's initial velocity

        // Ensures that when the robot starts the launchers are not already running
        towards_goal_launcher.setVelocity(STOP_SPEED);
        away_from_goal_launcher.setVelocity(STOP_SPEED);

    }

    @Override
    public void init_loop(){

        // Letting the user know that we are waiting for the launcher type
        telemetry.addData("Choose launcher type", "x for single, b for dual");

        // Assigning the launcher type based on user input

        if(gamepad1.x){ // <-- This checks if x has been pressed
            launcher_type = LauncherType.SINGLE; // Assigns the type as SINGLE meaning only one launcher will be running
        }
        if(gamepad1.b){ // <-- This checks if b has been pressed.
            launcher_type = LauncherType.DUAL; // Assigns the type as DUAL meaning both launchers will be running
        }

        if(gamepad1.dpad_up) {
            running_motor_velocity += 10;
            towards_goal_launcher_velocity += 10;
            away_from_goal_launcher_velocity += 10;
        }

        if (gamepad1.dpad_down) {
            running_motor_velocity -= 10;
            towards_goal_launcher_velocity -= 10;
            away_from_goal_launcher_velocity -= 10;
        }

        // Letting the user know either what type of launcher they have chosen or letting them know that they have not chosen one yet
        telemetry.addData("Chosen launcher type", launcher_type);
        telemetry.addData("Target Velocity", running_motor_velocity);
        telemetry.addData("Target Velocity", towards_goal_launcher_velocity);
        telemetry.addData("Target Velocity", away_from_goal_launcher_velocity);
        telemetry.update();
    }

    @Override
    public void loop(){

        switch(launcher_type) {

            // Code to run if only one launcher is running
            case SINGLE:

                // Changes which launcher is running based of user input
                if (gamepad1.left_bumper) {
                    running_motor = towards_goal_launcher; // Sets the running motor to the towards goal launcher
                    running_motor_velocity = towards_goal_launcher_velocity; // This is for telemetry and updating the velocities
                } else if (gamepad1.right_bumper) {
                    running_motor = away_from_goal_launcher; // Sets the running motor to the away from goal launcher
                    running_motor_velocity = away_from_goal_launcher_velocity; // This is for telemetry of the current running launcher
                }

                // Changes from SINGLE to DUAL if the user wants to try both launchers
                if (gamepad1.y) {
                    launcher_type = LauncherType.DUAL;
                }

                // Updates the velocity that the launcher is running at based on user input
                if (gamepad1.x) {
                    running_motor_velocity++;
                }

                if (gamepad1.b) {
                    running_motor_velocity--;
                }

                if (gamepad1.dpad_up) {
                    running_motor_velocity += 10;
                }

                if (gamepad1.dpad_down) {
                    running_motor_velocity -= 10;
                }

                // Updates the launcher's velocity with the new velocity (NOTE: the velocity may stay the same it doesn't have to change)
                running_motor.setVelocity(running_motor_velocity);

                // Making sure that the motor the user wants to test is the only one that is running
                if (running_motor == away_from_goal_launcher) {
                    away_from_goal_launcher_velocity = running_motor_velocity; // Making the actual motor's velocity equal to the running motor
                    towards_goal_launcher.setVelocity(STOP_SPEED);
                } else if (running_motor == towards_goal_launcher) {
                    towards_goal_launcher_velocity = running_motor_velocity; // Making the actual motor's velocity equal to the running motor
                    away_from_goal_launcher.setVelocity(STOP_SPEED);
                }



                // Giving the user all the information about the robot and the launchers right now
                telemetry.addData("Mode", "SINGLE (you can switch the launcher with LB and RB, x to increase velocity b to decrease velocity)");
                telemetry.addData("Current Launcher", running_motor.getDeviceName());
                telemetry.addData("Current Velocity For Launcher", running_motor.getVelocity());
                telemetry.addData("Target Velocity For Launcher", running_motor_velocity);
                telemetry.update();

                break;

            // Code to run if both launchers are running
            case DUAL:

                // Changing the velocity based on user input
                if (gamepad1.left_bumper) {
                    running_motor = towards_goal_launcher; // Sets the running motor to the towards goal launcher
                    running_motor_velocity = towards_goal_launcher_velocity; // This is for telemetry and updating the velocities
                } else if (gamepad1.right_bumper) {
                    running_motor = away_from_goal_launcher; // Sets the running motor to the away from goal launcher
                    running_motor_velocity = away_from_goal_launcher_velocity; // This is for telemetry of the current running launcher
                }

                // Changes from DUAL to SINGLE if the user wants to try only one launcher
                if(gamepad1.y){
                    launcher_type = LauncherType.SINGLE;
                }

                // Updates the velocity that the launchers are running at based on user input
                if(gamepad1.x){
                    running_motor_velocity++;
                }

                if(gamepad1.b){
                    running_motor_velocity--;
                }

                if (gamepad1.dpad_up) {
                    running_motor_velocity += 10;
                }

                if (gamepad1.dpad_down) {
                    running_motor_velocity -= 10;
                }

                // Making sure that the individual velocities are being affected/changed by the running velocity
                if(running_motor == away_from_goal_launcher){
                    away_from_goal_launcher_velocity = running_motor_velocity;
                } else if(running_motor == towards_goal_launcher){
                    towards_goal_launcher_velocity = running_motor_velocity;
                }


                // Updates both launchers velocities with the new velocity (NOTE: the velocity may stay the same it doesn't have to change)
                towards_goal_launcher.setVelocity(towards_goal_launcher_velocity);
                away_from_goal_launcher.setVelocity(away_from_goal_launcher_velocity);

                // Giving the user all the information about the robot and the launchers right now
                telemetry.addData("Mode", "DUAL (Both launchers should be running)");
                telemetry.addData("Current Velocity For Towards Goal Launcher", towards_goal_launcher.getVelocity());
                telemetry.addData("Target Velocity For Towards Goal Launcher", towards_goal_launcher_velocity);
                telemetry.addData("Current Velocity For Away From Goal Launcher", away_from_goal_launcher.getVelocity());
                telemetry.addData("Target Velocity For Away From Goal Launcher", away_from_goal_launcher_velocity);
                telemetry.update();

                break;

        }
    }

    @Override
    public void stop(){

        // Ensuring that when the robot stops the launchers are not still running
        towards_goal_launcher.setVelocity(STOP_SPEED);
        away_from_goal_launcher.setVelocity(STOP_SPEED);

    }
}