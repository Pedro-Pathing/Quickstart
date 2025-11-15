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

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Calculation_.Turret_Heading;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass;

//TODO, implement all our external libraries and functionality.

@TeleOp(name="Teleop_Main_", group="TeleOp")
public class Teleop_Main_ extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Contains the start Pose of our robot. This can be changed or saved from the autonomous period.
    private final Pose startPose = new Pose(135.6,9.8, Math.toRadians(90));

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Create a new instance of our Robot class
        LoadHardwareClass Robot = new LoadHardwareClass(this);
        Turret_Heading targeting = new Turret_Heading();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // This variable contains the target position for the turret.
        double target = 0;

        // Initialize all hardware of the robot
        Robot.init(startPose);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.left_trigger > 0.5){
                Robot.drivetrain.speedMultiplier = 0.2;
            }else{
                Robot.drivetrain.speedMultiplier = 1;
            }

            // Pass the joystick positions to our mecanum drive controller
            Robot.drivetrain.pedroMecanumDrive(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x/2,
                    true
            );

            if (gamepad2.guide){
                target = Math.abs(targeting.calcLocalizer(Robot.drivetrain.follower.getPose(), true)-540);
            }else if (gamepad2.back){
                target = Math.abs(targeting.calcLocalizer(Robot.drivetrain.follower.getPose(), false)-540);
            }else if (Math.abs(gamepad2.left_stick_x)>0.2 || Math.abs(gamepad2.left_stick_y)>0.2) {
                target = Math.toDegrees(Math.atan2(gamepad2.left_stick_y, gamepad2.left_stick_x));
            }

            Robot.turret.setAngle(target);

            if (gamepad2.a){
                Robot.intake.setPower(-1);
            }else if (gamepad2.x){
                Robot.intake.setPower(1);
            }

            Robot.belt.setPower(gamepad2.left_trigger);

            Robot.updatePIDs();

            // Turret-related Telemetry
            telemetry.addData("Turret PIDs", LoadHardwareClass.turretCoefficients);
            telemetry.addData("Turret Target Angle:", target);
            telemetry.addData("Turret Actual Angle", Robot.turret.getAngleAbsolute());
            telemetry.addData("Turret Set Power", Robot.turret.getPower());

            // Intake-related Telemetry
            telemetry.addLine();
            telemetry.addData("Intake Status", () -> {
                if(Robot.intake.getPower() == 1){
                    return "Outtaking";
                }else if (Robot.intake.getPower() == -1){
                    return "Intaking";
                }else{
                    return "Off";
                }
            });
            telemetry.addData("Intake RPM", Robot.intake.getRPM());

            // System-related Telemetry
            telemetry.addLine();
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Version: ", "11/4/25");
            telemetry.update();
            //TODO, Add a more advanced telemetry handler for better organization, readability, and debugging
        }
    }
}
