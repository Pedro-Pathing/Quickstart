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
 * CAUSED AND NEAR ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.LOADCode.Main_.Teleop_;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Intake;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Turret;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass;

@Configurable
@TeleOp(name="Teleop_Tuning_", group="TeleOp")
public class Teleop_Tuning_ extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();
    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    // Panels variables
    public static double hoodTargetPos = 0;
    public static int turretTurnIncrement = 2;
    int turretTurnMultiplier = 1;
    int turretTurnAngle = 0;

    // Contains the start Pose of our robot. This can be changed or saved from the autonomous period.
    private final Pose startPose = new Pose(88.5,7.8, Math.toRadians(90));

    @Override
    public void runOpMode() {

        // Create a new instance of our Robot class
        LoadHardwareClass Robot = new LoadHardwareClass(this);
        // Initialize all hardware of the robot
        Robot.init(startPose);

        while (opModeInInit() && Robot.turret.zeroTurret()){
            sleep(0);
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // Begin TeleOp driving
        Robot.drivetrain.startTeleOpDrive();
        Robot.turret.setGateState(Turret.gatestate.OPEN);

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
            if (gamepad1.x){
                Robot.turret.rotation.setAngle(turretTurnAngle);
                turretTurnAngle += turretTurnIncrement * turretTurnMultiplier;
                if (turretTurnAngle > 360){
                    turretTurnAngle = 360;
                    turretTurnMultiplier *= -1;
                }else if (turretTurnAngle < 0){
                    turretTurnAngle = 0;
                    turretTurnMultiplier *= -1;
                }

            }else if (gamepad1.b){
                Robot.turret.updateAimbotWithVelocity();
            }else if (gamepad1.a){
                Robot.turret.rotation.setAngle(0);
            }else{
                Robot.turret.rotation.setAngle(90);
            }
            Robot.turret.updatePIDs();
            telemetry.addLine();
            telemetry.addLine("TURRET DATA");
            telemetry.addData("Turret Target Angle", Robot.turret.rotation.target);
            telemetry.addData("Turret Actual Angle", Robot.turret.rotation.getAngleAbsolute());
            telemetry.addData("Hall Effect Detected", Robot.turret.hall.getTriggered());
            panelsTelemetry.addData("Turret Target Angle", Robot.turret.rotation.target);
            panelsTelemetry.addData("Turret Actual Angle", Robot.turret.rotation.getAngleAbsolute());


            // Controls for hood testing
//            if (gamepad1.dpad_up){
//                Robot.turret.setHood(Robot.turret.getHood() + 2);
//            }else if (gamepad1.dpad_down){
//                Robot.turret.setHood(Robot.turret.getHood() - 2);
//            }else if (gamepad1.dpadLeftWasPressed()){
//                Robot.turret.setHood(hoodTargetPos);
//            }
            telemetry.addLine();
            telemetry.addLine("HOOD DATA");
            telemetry.addData("Hood Angle", Robot.turret.getHood());

            // Controls for flywheel testing
            if (gamepad1.backWasPressed()){
                if (Robot.turret.flywheelMode == Turret.flywheelState.OFF){
                    Robot.turret.setFlywheelState(Turret.flywheelState.ON);
                }else{
                    Robot.turret.setFlywheelState(Turret.flywheelState.OFF);
                }
            }
            Robot.turret.updateFlywheel();
            telemetry.addLine();
            telemetry.addLine("FLYWHEEL DATA");
            telemetry.addData("Flywheel Target Velocity", Robot.turret.flywheel.target);
            telemetry.addData("Flywheel Actual Velocity", Robot.turret.getFlywheelRPM());
            telemetry.addData("Flywheel Motor Power", Robot.turret.flywheel.getPower());
            panelsTelemetry.addData("Flywheel Target Velocity", Robot.turret.flywheel.target);
            panelsTelemetry.addData("Flywheel Actual Velocity", Robot.turret.getFlywheelRPM());
            panelsTelemetry.addData("Flywheel Motor Power", Robot.turret.flywheel.getPower());

            if (gamepad1.aWasPressed()){
                if (Robot.intake.getMode() == Intake.intakeMode.INTAKING){
                    Robot.intake.setMode(Intake.intakeMode.OFF);
                }else{
                    Robot.intake.setMode(Intake.intakeMode.INTAKING);
                }
            }

            // System-related Telemetry
            telemetry.addLine();
            telemetry.addLine("SYSTEM DATA");
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Loop Time", loopTimer.toString());
            panelsTelemetry.addData("Loop Time", loopTimer.toString());
            telemetry.addData("Version: ", "12/26/25");
            telemetry.update();
            panelsTelemetry.update();
            loopTimer.reset();
        }
    }
}
