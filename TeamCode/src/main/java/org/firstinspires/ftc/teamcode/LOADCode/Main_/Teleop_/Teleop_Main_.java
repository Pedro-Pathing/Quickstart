/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.LOADCode.Main_.Teleop_;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Intake;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass;

//TODO, implement all our external libraries and functionality.

@TeleOp(name="Teleop_Main_", group="TeleOp")
public class Teleop_Main_ extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    Prompter prompter = null;

    // Contains the start Pose of our robot. This can be changed or saved from the autonomous period.
    private final Pose startPose = new Pose(135.6,9.8, Math.toRadians(90));

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Create a new instance of our Robot class
        LoadHardwareClass Robot = new LoadHardwareClass(this);

        // Create a new prompter for selecting alliance
        prompter = new Prompter(this);
        prompter.prompt("alliance", new OptionPrompt<>("Select Alliance", LoadHardwareClass.Alliance.RED, LoadHardwareClass.Alliance.BLUE));
        prompter.onComplete(() -> {
                    LoadHardwareClass.selectedAlliance = prompter.get("alliance");
                    telemetry.addData("Selection", "Complete");
                }
        );

        // Runs repeatedly while in init
        while (opModeInInit()){
            // If an auto was not run, run the prompter to select the correct alliance
            if (LoadHardwareClass.selectedAlliance == null){
                prompter.run();
            }
        }

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // Initialize all hardware of the robot
        Robot.init(startPose);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            Robot.drivetrain.speedMultiplier = 0.75;
            if (gamepad1.left_trigger >= 0.5) {
                Robot.drivetrain.speedMultiplier -= 0.25;
            }
            if (gamepad1.right_trigger >= 0.5){
                Robot.drivetrain.speedMultiplier += 0.25;
            }

            // Pass the joystick positions to our mecanum drive controller
            Robot.drivetrain.pedroMecanumDrive(
                    Math.pow(gamepad1.left_stick_y,2),
                    Math.pow(gamepad1.left_stick_x,2),
                    Math.pow(gamepad1.right_stick_x/2,2),
                    true
            );

            if (gamepad2.guide){
                Robot.turret.updateAimbot(Robot.drivetrain.follower.getPose(), true);
            }else if (gamepad2.back){
                Robot.turret.updateAimbot(Robot.drivetrain.follower.getPose(), false);
            }else if (Math.abs(gamepad2.left_stick_x)>0.2 || Math.abs(gamepad2.left_stick_y)>0.2) {
                Robot.turret.rotation.setAngle(
                        Math.toDegrees(Math.atan2(gamepad2.left_stick_y, gamepad2.left_stick_x)));
            }

            if (gamepad2.a){
                Robot.intake.setMode(Intake.Mode.INTAKING);
            }else if (gamepad2.b){
                Robot.intake.setMode(Intake.Mode.SHOOTING);
            }else if (gamepad2.x){
                Robot.intake.setMode(Intake.Mode.REVERSING);
            }else{
                Robot.intake.setMode(Intake.Mode.OFF);
            }

            Robot.turret.updatePIDs();

            // Turret-related Telemetry
            telemetry.addData("Turret Target Angle:", Robot.turret.rotation.target);
            telemetry.addData("Turret Actual Angle", Robot.turret.rotation.getAngleAbsolute());

            // Intake-related Telemetry
            telemetry.addLine();
            telemetry.addData("Intake Status", Robot.intake.getMode());

            // System-related Telemetry
            telemetry.addLine();
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Version: ", "11/4/25");
            telemetry.update();
            //TODO, Add a more advanced telemetry handler for better organization, readability, and debugging
        }
    }
}
