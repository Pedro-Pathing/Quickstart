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

import static org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass.selectedAlliance;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.skeletonarmy.marrow.TimerEx;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Intake;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Intake.intakeMode;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Intake.transferState;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Turret;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Turret.flywheelState;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Turret.gatestate;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Drivetrain_.MecanumDrivetrainClass;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Drivetrain_.Pedro_Paths;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass;


// FIRST COMMENT FROM MY NEW COMPUTER =D
// - ARI

@Configurable
@TeleOp(name="Teleop_Main_", group="TeleOp")
public class Teleop_Main_ extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    // Variables for storing data for the second gamepad controls
    public static double DylanStickDeadzones = 0.2;

    public int shootingState = 0;
    public boolean turretOn = true;
    public TimerEx stateTimer = new TimerEx(1);
    public double hoodOffset = 0;

    // Create a new instance of our Robot class
    LoadHardwareClass Robot = new LoadHardwareClass(this);
    // Create a new Paths instance
    Pedro_Paths Paths = new Pedro_Paths();
    // Create a new instance of Prompter for selecting the alliance
    Prompter prompter = null;
    enum startPoses {
        FAR,
        NEAR
    }

    // Contains the start Pose of our robot. This can be changed or saved from the autonomous period.
    private Pose startPose = Paths.farStart;

    @Override
    public void runOpMode() {

        // Create a new prompter for selecting alliance
        prompter = new Prompter(this);
        prompter.prompt("alliance", () -> {
            if (selectedAlliance == null){
                return new OptionPrompt<>("Select Alliance", LoadHardwareClass.Alliance.RED, LoadHardwareClass.Alliance.BLUE);
            }else{
                return null;
            }
        });
        prompter.prompt("startPose", () -> {
            if (MecanumDrivetrainClass.robotPose == null){
                return new OptionPrompt<>("Select Start Pose",
                        startPoses.FAR,
                        startPoses.NEAR
                        );
            }else{
                startPose = MecanumDrivetrainClass.robotPose;
                return null;
            }
        });
        prompter.onComplete(() -> {
            if (selectedAlliance == null){
                selectedAlliance = prompter.get("alliance");
            }
            telemetry.addData("Selection", "Complete");
            telemetry.addData("Alliance", selectedAlliance);
            if (MecanumDrivetrainClass.robotPose == null){
                startPoses pose = prompter.get("startPose");
                if (pose.equals(startPoses.FAR)) {
                    startPose = Paths.autoMirror(Paths.farStart, selectedAlliance);
                    telemetry.addData("Start Pose", "Far Start Pose");
                } else if (pose.equals(startPoses.NEAR)) {
                    startPose = Paths.autoMirror(Paths.nearStart, selectedAlliance);
                    telemetry.addData("Start Pose", "Near Start Pose");
                }
            }else{
                startPose = MecanumDrivetrainClass.robotPose;
                telemetry.addData("Start Pose", "Ending Pose of Auto");
            }
            telemetry.update();
        });

        // Runs repeatedly while in init
        while (opModeInInit()) {
            prompter.run();
        }

        // Wait for the game to start (driver presses START)
        waitForStart();
        // Initialize all hardware of the robot
        Robot.init(startPose);
        runtime.reset();
        Paths.buildPaths(selectedAlliance, Robot.drivetrain.follower);
        Robot.drivetrain.startTeleOpDrive();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (!Turret.zeroed){
                while (!isStopRequested() && Robot.turret.zeroTurret()){
                    sleep(0);
                }
            }

            Gamepad1();
            Gamepad2();

            Robot.turret.updatePIDs();

            double flywheelPercentage = (int) Math.round(Robot.turret.getFlywheelRPM()/Robot.turret.getFlywheelCurrentMaxSpeed() *100);
            telemetry.addData("Flywheel Percentage", flywheelPercentage+"%");
            panelsTelemetry.addData("Flywheel Percentage", flywheelPercentage+"%");

            telemetry.addData("SpeedMult", Robot.drivetrain.speedMultiplier);
            telemetry.addLine();
            //positional telemetry
            telemetry.addData("X Position", Robot.drivetrain.follower.getPose().getX());
            telemetry.addData("Y Position", Robot.drivetrain.follower.getPose().getY());
            telemetry.addData("Heading", Math.toDegrees(Robot.drivetrain.follower.getPose().getHeading()));
            telemetry.addData("Distance From Goal", Robot.drivetrain.distanceFromGoal());

            telemetry.addLine();
            // Turret-related Telemetry
            panelsTelemetry.addData("Turret Target Angle", Robot.turret.rotation.target);
            panelsTelemetry.addData("Turret Actual Angle", Robot.turret.rotation.getAngleAbsolute());
            telemetry.addData("Turret Target Angle", Robot.turret.rotation.target);
            telemetry.addData("Turret Actual Angle", Robot.turret.rotation.getAngleAbsolute());
            telemetry.addData("Turret Hood Angle", Robot.turret.getHood());

            telemetry.addLine();
            panelsTelemetry.addData("Flywheel Target Speed", Robot.turret.flywheel.target);
            panelsTelemetry.addData("Flywheel Actual Speed", Robot.turret.getFlywheelRPM());
            panelsTelemetry.addData("Flywheel Power", Robot.turret.flywheel.getPower());
            telemetry.addData("Flywheel Target Speed", Robot.turret.flywheel.target);
            telemetry.addData("Flywheel Actual Speed", Robot.turret.getFlywheelRPM());
            telemetry.addData("Flywheel State", Robot.turret.getFlywheelRPM());

            // Intake-related Telemetry
            telemetry.addLine();
            telemetry.addData("Intake Mode", Robot.intake.getMode());

            // Color Sensor Telemetry
            telemetry.addLine();
            telemetry.addData("Upper Sensor", Robot.intake.getTopSensorState());
            telemetry.addData("Lower Sensor", Robot.intake.getBottomSensorState());

            // System-related Telemetry
            telemetry.addLine();
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Version: ", "12/12/25");
            telemetry.update();
            panelsTelemetry.update();
        }
    }

    /**
     * <h1>Gamepad 1 Controls (Ari's Pick V1)</h1>
     * <ul>
     *     <li><b>Analog Inputs</b><ul>
     *         <li>Left Stick:<ul>
     *             <li>X: <code>Rotate Left/Right</code></li>
     *             <li>Y: <code>N/A</code></li>
     *         </ul></li>
     *         <li>Right Stick:<ul>
     *             <li>X: <code>Strafe Left/Right</code></li>
     *             <li>Y: <code>Drive Forwards/Backwards</code></li>
     *         </ul></li>
     *         <li>Left Trigger: <code>Slow Mod</code></li>
     *         <li>Right Trigger: <code>Quick Mod</code></li>
     *     </ul></li>
     *
     *     <li><b>Button Inputs</b></li><ul>
     *         <li>Letter Buttons:<ul>
     *             <li>A: <code>N/A</code></li>
     *             <li>B: <code>N/A</code></li>
     *             <li>X: <code>N/A</code></li>
     *             <li>Y: <code>N/A</code></li>
     *         </ul></li>
     *         <li>Letter Buttons:<ul>
     *             <li>DpadUp: <code>N/A</code></li>
     *             <li>DpadDown: <code>N/A</code></li>
     *             <li>DpadLeft: <code>N/A</code></li>
     *             <li>DpadRight: <code>N/A</code></li>
     *         </ul></li>
     *         <li>Bumpers:<ul>
     *             <li>Left Bumper: <code>N/A</code></li>
     *             <li>Right Bumper: <code>N/A</code></li>
     *         </ul></li>
     *         <li>Stick Buttons:<ul>
     *             <li>Left Stick Button: <code>N/A</code></li>
     *             <li>Right Stick Button: <code>N/A</code></li>
     *         </ul></li>
     *     </ul>
     * </ul>
     */
    public void Gamepad1() {

        if (gamepad1.left_trigger >= 0.5 && gamepad1.right_trigger >= 0.5) {
            Robot.drivetrain.speedMultiplier = 0.66;
        } else if (gamepad1.left_trigger >= 0.5) {
            Robot.drivetrain.speedMultiplier = 0.33;
        } else if (gamepad1.right_trigger >= 0.5) {
            Robot.drivetrain.speedMultiplier = 1;
        } else {
            Robot.drivetrain.speedMultiplier = 0.66;
        }

        if (gamepad1.bWasPressed()){
            if (selectedAlliance == LoadHardwareClass.Alliance.RED){
                Robot.drivetrain.follower.setPose(new Pose(8, 8, Math.toRadians(90)));
            }else if (selectedAlliance == LoadHardwareClass.Alliance.BLUE){
                Robot.drivetrain.follower.setPose(new Pose(136, 8, Math.toRadians(90)));
            }
        }

        double turnMult = 2;
//        if (gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0){
//            turnMult = 1;
//        }
        Robot.drivetrain.pedroMecanumDrive(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x / turnMult,
                true
        );
    }

    /**
     * <h1>Gamepad 2 Controls (Dylan's Pick V1)</h1>
     * <ul>
     *     <li><b>Analog Inputs</b><ul>
     *         <li>Left Stick:<ul>
     *             <li>X: <code>N/A</code></li>
     *             <li>Y: <code>Intake IN</code></li>
     *         </ul></li>
     *         <li>Right Stick:<ul>
     *             <li>X: <code>Belt IN</code></li>
     *             <li>Y: <code>N/A</code></li>
     *         </ul></li>
     *         <li>Left Trigger: <code>N/A</code></li>
     *         <li>Right Trigger: <code>N/A</code></li>
     *     </ul></li>
     *
     *     <li><b>Button Inputs</b></li><ul>
     *         <li>Letter Buttons:<ul>
     *             <li>A: <code>Toggle Turret Autoaim (Default on, locks to forward when off</code></li>
     *             <li>B: <code>Shoot</code></li>
     *             <li>X: <code>N/A</code></li>
     *             <li>Y: <code>Flywheel Toggle</code></li>
     *         </ul></li>
     *         <li>Letter Buttons:<ul>
     *             <li>DpadUp: <code>Hood Up (Does not work currently)</code></code></li>
     *             <li>DpadDown: <code>Hood Down (Does not work currently)</code></li>
     *             <li>DpadLeft: <code>Sets hood to ideal angle for far zone shooting</code></li>
     *             <li>DpadRight: <code>N/A</code></li>
     *         </ul></li>
     *         <li>Bumpers:<ul>
     *             <li>Left Bumper: <code>Transfer Belt Out</code></li>
     *             <li>Right Bumper: <code>Transfer Belt In</code></li>
     *         </ul></li>
     *         <li>Stick Buttons:<ul>
     *             <li>Left Stick Button: <code>N/A</code></li>
     *             <li>Right Stick Button: <code>N/A</code></li>
     *         </ul></li>
     *         <li>Other Buttons:<ul>
     *             <li>Back Button: <code>Intake Reverse</code></li>
     *         </ul></li>
     *     </ul>
     * </ul>
     */
    public void Gamepad2() {

        // Turret Aimbot
        if (gamepad2.aWasPressed()){
            turretOn = !turretOn;
        }
        Robot.turret.updateAimbot(turretOn, true, hoodOffset);

        //Intake Controls (Left Stick Y)
        if (shootingState == 0) {
            if (Math.abs(gamepad2.left_stick_y) >= DylanStickDeadzones &&
                    Math.abs(gamepad2.right_stick_y) >= DylanStickDeadzones) {
                Robot.intake.setMode(intakeMode.INTAKING);
            }else if (Math.abs(gamepad2.left_stick_y) >= DylanStickDeadzones &&
                    Math.abs(gamepad2.right_stick_y) < DylanStickDeadzones) {
                Robot.intake.setMode(intakeMode.NO_BELT);
            }else if (Math.abs(gamepad2.left_stick_y) < DylanStickDeadzones &&
                    Math.abs(gamepad2.right_stick_y) >= DylanStickDeadzones) {
                Robot.intake.setMode(intakeMode.SHOOTING);
            }else if (gamepad2.back){
                Robot.intake.setMode(intakeMode.REVERSING);
            }else{ // OFF
                Robot.intake.setMode(intakeMode.OFF);
            }

            /* TODO Uncomment once autobelt control is finished
            if (Math.abs(gamepad2.left_stick_y) >= DylanStickDeadzones) {
                Robot.intake.setMode(intakeMode.INTAKING);
            }else{ // OFF
                Robot.intake.setMode(intakeMode.OFF);
            }
             */

            //Flywheel Toggle Control (Y Button)
            if (gamepad2.yWasPressed()) {
                if (Robot.turret.flywheelMode == flywheelState.OFF) {
                    Robot.turret.setFlywheelState(flywheelState.ON);
                } else {
                    Robot.turret.setFlywheelState(flywheelState.OFF);
                }
            }
        }
        Robot.turret.updateFlywheel();

        // Hood Controls
        if (gamepad2.dpadUpWasPressed()){
            hoodOffset += 10;
        }else if (gamepad2.dpadDownWasPressed()){
            hoodOffset -= 10;
        }


        //Shoot (B Button Press)
        // Increment the shooting state
        if (gamepad2.bWasPressed() && shootingState < 1 && Robot.turret.getFlywheelRPM() > Robot.turret.getFlywheelCurrentMaxSpeed()-100) {
            shootingState++;
        }
        switch (shootingState) {
            case 0:
                telemetry.addData("Shooting State", "OFF");
                return;
            case 1:
                if (Robot.intake.getMode() == intakeMode.OFF){
                    stateTimer.restart();
                    stateTimer.start();
                }
                Robot.intake.setMode(intakeMode.INTAKING);
                Robot.turret.setGateState(gatestate.OPEN);
                telemetry.addData("Shooting State", "STARTED");
                if (stateTimer.isDone() && Robot.intake.getTopSensorState() && !Robot.intake.getBottomSensorState()){
                    shootingState++;
                }
                return;
            case 2:
                if (Robot.intake.getMode() == intakeMode.INTAKING){
                    stateTimer.restart();
                    stateTimer.start();
                }
                Robot.intake.setMode(Intake.intakeMode.SHOOTING);
                Robot.intake.setTransfer(transferState.UP);
                telemetry.addData("Shooting State", "TRANSFERRED");
                if (stateTimer.isDone()) {
                    shootingState++;
                }
                return;
            case 3:
                Robot.turret.setFlywheelState(flywheelState.OFF);
                Robot.turret.setGateState(gatestate.CLOSED);
                Robot.intake.setMode(intakeMode.OFF);
                Robot.intake.setTransfer(transferState.DOWN);
                telemetry.addData("Shooting State", "RESET");
                shootingState = 0;
        }
    }
}