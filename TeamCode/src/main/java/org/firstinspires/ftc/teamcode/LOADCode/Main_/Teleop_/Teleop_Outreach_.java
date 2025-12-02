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

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass;

//TODO, implement all our external libraries and functionality.

@TeleOp(name="Teleop_Outreach_", group="TeleOp")
public class Teleop_Outreach_ extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    // Contains the start Pose of our robot. This can be changed or saved from the autonomous period.
    private final Pose startPose = new Pose(135.6,9.8, Math.toRadians(90));

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Create a new instance of our Robot class
        LoadHardwareClass Robot = new LoadHardwareClass(this);

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // Initialize all hardware of the robot
        Robot.init(startPose);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Pass the joystick positions to our mecanum drive controller
            Robot.drivetrain.pedroMecanumDrive(
                    gamepad1.left_stick_y/2,
                    gamepad1.left_stick_x/2,
                    gamepad1.right_stick_x/2,
                    true
            );
            if (gamepad1.x){
                Robot.turret.setFlywheelRPM(5485.714285714286);
                panelsTelemetry.addData("SetRPM", 5485.714285714286);
            }else{
                Robot.turret.setFlywheelRPM(0);
                panelsTelemetry.addData("SetRPM", 0);
            }

            Robot.turret.updatePIDs();

            telemetry.addData("FlywheelState", Robot.turret.flywheelState);
            panelsTelemetry.addData("FlywheelRPM", Robot.turret.getFlywheelRPM());


            // System-related Telemetry
            telemetry.addLine();
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Version: ", "11/4/25");
            telemetry.update();
            panelsTelemetry.update(telemetry);
            //TODO, Add a more advanced telemetry handler for better organization, readability, and debugging
        }
    }
}
