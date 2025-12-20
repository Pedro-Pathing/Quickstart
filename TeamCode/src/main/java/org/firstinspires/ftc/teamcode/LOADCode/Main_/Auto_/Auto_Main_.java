package org.firstinspires.ftc.teamcode.LOADCode.Main_.Auto_;

import static org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass.selectedAlliance;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Intake;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Turret;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Commands;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Drivetrain_.Pedro_Paths;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Auto_Main_", group = "Main", preselectTeleOp="Teleop_Main_")
public class Auto_Main_ extends NextFTCOpMode {

    // Define other PedroPathing constants
    private Pose startPose = new Pose(87, 8.8, Math.toRadians(90)); // Start Pose of our robot.

    private enum Auto {
        LEAVE_NEAR_LAUNCH,
        LEAVE_FAR_HP,
        TEST_AUTO
    }

    private Auto selectedAuto = null;
    private boolean turretOn = true;

    // Create the prompter object for selecting Alliance and Auto
    Prompter prompter = null;

    // Create a new instance of our Robot class
    LoadHardwareClass Robot = new LoadHardwareClass(this);
    // Create a Paths object for accessing modular auto paths
    Pedro_Paths paths = new Pedro_Paths();

    public Auto_Main_() {
        addComponents(
                new PedroComponent(Constants::createFollower)
        );
    }

    @Override
    public void onInit() {
        prompter = new Prompter(this);
        prompter.prompt("alliance",
                new OptionPrompt<>("Select Alliance",
                        LoadHardwareClass.Alliance.RED,
                        LoadHardwareClass.Alliance.BLUE
                ));
        prompter.prompt("auto",
                new OptionPrompt<>("Select Auto",
                        Auto.LEAVE_NEAR_LAUNCH,
                        Auto.LEAVE_FAR_HP,
                        Auto.TEST_AUTO
                ));
        prompter.onComplete(() -> {
                    selectedAlliance = prompter.get("alliance");
                    selectedAuto = prompter.get("auto");
                    telemetry.addData("Selection", "Complete");
                    telemetry.addData("Alliance", selectedAlliance);
                    telemetry.addData("Auto", selectedAuto);
                    telemetry.update();
                }
        );
    }

    @Override
    public void onWaitForStart() {
        prompter.run();
    }

    @Override
    public void onStartButtonPressed() {
        // Build paths
        paths.buildPaths(selectedAlliance, follower());
        // Schedule the proper auto
        switch (selectedAuto) {
            case LEAVE_NEAR_LAUNCH:
                Leave_Near_Launch().schedule();
                break;
            case LEAVE_FAR_HP:
                Leave_Far_HP().schedule();
                break;
            case TEST_AUTO:
                test_Auto().schedule();
                break;
        }
        // Initialize all hardware of the robot
        Robot.init(startPose, follower());
        telemetry.addData("Initialized", "");
        telemetry.update();
    }

    @Override
    public void onUpdate() {
        if (turretOn){
            switch (selectedAlliance) {
                case RED:
                    telemetry.addData("Aimbot Target", "RED");
                    Robot.turret.updateAimbot(Robot, true);
                    break;
                case BLUE:
                    telemetry.addData("Aimbot Target", "BLUE");
                    Robot.turret.updateAimbot(Robot, false);
                    break;
            }
        }

        Robot.turret.updateFlywheel();

        telemetry.addData("Path", paths.farStart_to_farLeave.endPose());
        telemetry.addLine();
        telemetry.addData("selectedAuto", selectedAuto);
        telemetry.addData("currentPose", Robot.drivetrain.follower.getPose());
        telemetry.update();
    }

    /**
     * This auto starts at the far zone,
     * shoots it's preloads after 5 seconds,
     * and goes to the leave zone next to the human player zone.
     */
    private Command Leave_Far_HP() {
        turretOn = true;
        startPose = paths.farStart;
        return new SequentialGroup(
                new ParallelGroup(
                        // Commands.setFlywheelState(Robot, Turret.flywheelstate.ON),
                        new Delay(5)
                ),
                Commands.setIntakeMode(Robot, Intake.intakeMode.SHOOTING),
                new Delay(2),
                Commands.setIntakeMode(Robot, Intake.intakeMode.INTAKING),
                new Delay(1),
                Commands.setIntakeMode(Robot, Intake.intakeMode.SHOOTING),
                new Delay(2),
                Commands.setTransferState(Robot, Intake.transferState.UP),
                new Delay(1),
                Commands.setTransferState(Robot, Intake.transferState.DOWN),
                Commands.setIntakeMode(Robot, Intake.intakeMode.OFF),
                Commands.setFlywheelState(Robot, Turret.flywheelstate.OFF),
                Commands.runPath(paths.farStart_to_farLeave, true, 0.6)
        );
    }

    /**
     * This auto starts at the near zone,
     * shoots it's preloads after 5 seconds,
     * and goes to the leave pose that is in the launch zone.
     */
    private Command Leave_Near_Launch() {
        turretOn = true;
        startPose = paths.nearStart;
        return new SequentialGroup(
                Commands.setFlywheelState(Robot, Turret.flywheelstate.ON),
                new ParallelGroup(
                        new WaitUntil(() -> Robot.turret.getFlywheelRPM() > Turret.onSpeed),
                        new Delay(5)
                ),
                Commands.setIntakeMode(Robot, Intake.intakeMode.SHOOTING),
                new Delay(2),
                Commands.setIntakeMode(Robot, Intake.intakeMode.INTAKING),
                new Delay(1),
                Commands.setIntakeMode(Robot, Intake.intakeMode.SHOOTING),
                new Delay(2),
                Commands.setTransferState(Robot, Intake.transferState.UP),
                new Delay(1),
                Commands.setTransferState(Robot, Intake.transferState.DOWN),
                Commands.setIntakeMode(Robot, Intake.intakeMode.OFF),
                Commands.setFlywheelState(Robot, Turret.flywheelstate.OFF),
                Commands.runPath(paths.nearShoot_to_nearLeave, true, 0.6)
        );
    }

    private Command test_Auto(){
        return Commands.runPath(paths.farStart_to_nearPreload, true);
    }
}
