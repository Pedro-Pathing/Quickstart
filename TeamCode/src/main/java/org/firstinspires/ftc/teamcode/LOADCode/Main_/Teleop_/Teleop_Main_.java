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

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.Drivetrain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Intake;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Turret;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass;


@Configurable
@TeleOp(name="Teleop_Main_", group="TeleOp")
public class Teleop_Main_ extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    public static double driveExponent = 2;

    Prompter prompter = null;

    // Contains the start Pose of our robot. This can be changed or saved from the autonomous period.
    private final Pose startPose = new Pose(135.6,9.8, Math.toRadians(90));

    // Create a new instance of our Robot class
    LoadHardwareClass Robot = new LoadHardwareClass(this);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


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

            Gamepad1();
            Gamepad2();

            Robot.turret.updatePIDs();

            telemetry.addData("SpeedMult", Robot.drivetrain.speedMultiplier);
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
    public void Gamepad1(){

        Robot.drivetrain.speedMultiplier = 0.66;
        if (gamepad1.left_trigger >= 0.5) {
            Robot.drivetrain.speedMultiplier -= 0.33;
        }
        if (gamepad1.right_trigger >= 0.5){
            Robot.drivetrain.speedMultiplier += 0.33;
        }

        double turnMult = 2;
        if (gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0){
            turnMult = 1;
        }
        Robot.drivetrain.pedroMecanumDrive(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x/turnMult,
                true
        );
    }

    /**
     * <h1>Gamepad 2 Controls (Dylan's Pick V1)</h1>
     * <ul>
     *     <li><b>Analog Inputs</b><ul>
     *         <li>Left Stick:<ul>
     *             <li>X: <code>N/A</code></li>
     *             <li>Y: <code>Intake Direction/Power</code></li>
     *         </ul></li>
     *         <li>Right Stick:<ul>
     *             <li>X: <code>Turret Angle Override</code></li>
     *             <li>Y: <code>N/A</code></li>
     *         </ul></li>
     *         <li>Left Trigger: <code>N/A</code></li>
     *         <li>Right Trigger: <code>N/A</code></li>
     *     </ul></li>
     *
     *     <li><b>Button Inputs</b></li><ul>
     *         <li>Letter Buttons:<ul>
     *             <li>A: <code>N/A</code></li>
     *             <li>B: <code>Shoot</code></li>
     *             <li>X: <code>N/A</code></li>
     *             <li>Y: <code>Flywheel Toggle</code></li>
     *         </ul></li>
     *         <li>Letter Buttons:<ul>
     *             <li>DpadUp: <code>Hood Up Override</code></li>
     *             <li>DpadDown: <code>Hood Down Override</code></li>
     *             <li>DpadLeft: <code>N/A</code></li>
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
     *     </ul>
     * </ul>
     */
    public void Gamepad2(){
        double DylanStickDeadzones = 0.2;

        //Intake Controls (Left Stick Y)
        if (Math.abs(gamepad2.left_stick_y) >= DylanStickDeadzones){
            if (gamepad2.left_stick_y > 0){ // OUT (Digital)
                Robot.intake.setMode(Intake.Mode.REVERSING);
            } else { // IN (Digital)
                Robot.intake.setMode(Intake.Mode.INTAKING);
            }
        } else { // OFF
            Robot.intake.setMode(Intake.Mode.OFF);
        }

        //Turret Angle Controls (Right Stick X)
        //To be added after manual control is finished

        //Flywheel Toggle Control (Y Button)

    }
}
