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

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Intake;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Turret;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass;

@TeleOp(name="Teleop_Tuning_", group="TeleOp")
public class Teleop_Tuning_ extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    // Contains the start Pose of our robot. This can be changed or saved from the autonomous period.
    private final Pose startPose = new Pose(88.5,7.8, Math.toRadians(90));

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Create a new instance of our Robot class
        LoadHardwareClass Robot = new LoadHardwareClass(this);
        // Initialize all hardware of the robot
        Robot.init(startPose);

        if (gamepad1.guide){
            Robot.turret.rotation.resetEncoder();
        }

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // Begin TeleOp driving
        Robot.drivetrain.startTeleOpDrive();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Pass the joystick positions to our mecanum drive controller
            Robot.drivetrain.pedroMecanumDrive(
                    gamepad1.left_stick_y/2,
                    gamepad1.left_stick_x/2,
                    gamepad1.right_stick_x/2,
                    true
            );

            // Color sensor telemetry
            telemetry.addLine("COLOR SENSOR DATA");
            telemetry.addData("Color Sensor Threshold", Intake.proximitySensorThreshold);
            telemetry.addData("Upper Sensor Array Triggered", Robot.intake.getTopSensorState());
            telemetry.addData("Lower Sensor Array Triggered", Robot.intake.getBottomSensorState());

            // Controls for turret rotation testing
            if (!gamepad1.x){
                Robot.turret.rotation.setAngle(0);
            }else{
                Robot.turret.rotation.setAngle(180);
            }

            // Controls for hood testing
            if (gamepad1.dpad_up){
                Robot.turret.setHood(Robot.turret.getHood() + 2);
            }else if (gamepad1.dpad_down){
                Robot.turret.setHood(Robot.turret.getHood() - 2);
            }
            telemetry.addLine();
            telemetry.addLine("HOOD DATA");
            telemetry.addData("Hood Angle", Robot.turret.getHood());

            // Controls for flywheel testing
            if (gamepad1.yWasPressed()){
                if (Robot.turret.flywheelState == Turret.flywheelstate.OFF){
                    Robot.turret.setFlywheelState(Turret.flywheelstate.ON);
                }else{
                    Robot.turret.setFlywheelState(Turret.flywheelstate.OFF);
                }
            }
            Robot.turret.updateFlywheel();
            telemetry.addLine();
            telemetry.addLine("FLYWHEEL DATA");
            telemetry.addData("Flywheel Target Velocity", Robot.turret.flywheel.target);
            telemetry.addData("Flywheel Actual Velocity", Robot.turret.getFlywheelRPM());
            panelsTelemetry.addData("Flywheel Target Velocity", Robot.turret.flywheel.target);
            panelsTelemetry.addData("Flywheel Actual Velocity", Robot.turret.getFlywheelRPM());


            // System-related Telemetry
            telemetry.addLine();
            telemetry.addLine("SYSTEM DATA");
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Version: ", "12/26/25");
            telemetry.update();
            panelsTelemetry.update();
        }
    }
}
