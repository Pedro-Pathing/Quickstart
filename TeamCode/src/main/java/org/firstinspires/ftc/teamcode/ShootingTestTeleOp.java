package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * TeleOp for testing shooting sequences
 * 
 * Controls:
 * A - SSS (Shoot, Shoot, Shoot)
 * B - ISISS (Index, Shoot, Index, Shoot, Shoot)
 * X - ISSIS (Index, Shoot, Shoot, Index, Shoot)
 * Y - SISIS (Shoot, Index, Shoot, Index, Shoot)
 * 
 * Right Bumper - Intake ON (hold)
 */
@TeleOp(name = "Shooting Test TeleOp", group = "Test")
public class ShootingTestTeleOp extends LinearOpMode {

    private HardwareConfigAuto robot;
    private RobotFunctionsAuto robotFunctions;
    
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        robot = new HardwareConfigAuto();
        robot.init(hardwareMap);
        robotFunctions = new RobotFunctionsAuto(robot);
        
        // Set initial states
        robotFunctions.setBlocker(true);
        
        telemetry.addLine("=== Shooting Test TeleOp ===");
        telemetry.addLine("A = SSS");
        telemetry.addLine("B = ISISS");
        telemetry.addLine("X = ISSIS");
        telemetry.addLine("Y = SISIS");
        telemetry.addLine("RB = Intake");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // === INTAKE CONTROL ===
            if (gamepad1.right_bumper) {
                robotFunctions.runIntakeSystem(HardwareConfigAuto.INTAKE_POWER);
            } else if (!gamepad1.a && !gamepad1.b && !gamepad1.x && !gamepad1.y) {
                robotFunctions.stopIntakeSystem();
            }
            
            // === SHOOTING SEQUENCES ===
            // A - SSS
            if (gamepad1.a && !lastA) {
                telemetry.addLine(">>> Running SSS...");
                telemetry.update();
                robotFunctions.SSS(this);
            }
            
            // B - ISISS
            if (gamepad1.b && !lastB) {
                telemetry.addLine(">>> Running ISISS...");
                telemetry.update();
                robotFunctions.ISISS(this);
            }
            
            // X - ISSIS
            if (gamepad1.x && !lastX) {
                telemetry.addLine(">>> Running ISSIS...");
                telemetry.update();
                robotFunctions.ISSIS(this);
            }
            
            // Y - SISIS
            if (gamepad1.y && !lastY) {
                telemetry.addLine(">>> Running SISIS...");
                telemetry.update();
                robotFunctions.SISIS(this);
            }
            
            // Update button states
            lastA = gamepad1.a;
            lastB = gamepad1.b;
            lastX = gamepad1.x;
            lastY = gamepad1.y;
            
            // === TELEMETRY ===
            telemetry.addLine("=== Shooting Test TeleOp ===");
            telemetry.addLine("A = SSS | B = ISISS");
            telemetry.addLine("X = ISSIS | Y = SISIS");
            telemetry.addLine("RB = Intake");
            telemetry.update();
        }
    }
}
