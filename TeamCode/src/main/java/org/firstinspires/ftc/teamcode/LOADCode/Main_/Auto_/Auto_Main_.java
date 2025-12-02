package org.firstinspires.ftc.teamcode.LOADCode.Main_.Auto_;

import static org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass.selectedAlliance;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Intake;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Turret;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Commands;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Disabled
@Autonomous(name = "Auto_Main_", group = "Main", preselectTeleOp="Teleop_Main_")
public class Auto_Main_ extends NextFTCOpMode {

    // Define other PedroPathing constants
    private final Pose startPose = new Pose(87, 8.8, Math.toRadians(90)); // Start Pose of our robot.

    private enum Auto {
        LEAVE_NEAR_LAUNCH,
        LEAVE_FAR_HP,
        MOVEMENT_RP_DEC6
    }

    private Auto selectedAuto = null;
    private boolean turretOn = false;

    // Create the prompter object for selecting Alliance and Auto
    Prompter prompter = null;

    // Create a new instance of our Robot class
    LoadHardwareClass Robot = new LoadHardwareClass(this);

    @Override
    public void onInit() {
        // Initialize all hardware of the robot
        addComponents(
                new PedroComponent(Constants::createFollower)
        );
        Robot.init(startPose);
        Robot.drivetrain.follower = PedroComponent.follower();

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
                        Auto.MOVEMENT_RP_DEC6
                ));
        prompter.onComplete(() -> {
                    selectedAlliance = prompter.get("alliance");
                    selectedAuto = prompter.get("auto");
                    telemetry.addData("Selection", "Complete");
                }
        );
        Robot.drivetrain.paths.buildPaths(selectedAlliance);
    }

    @Override
    public void onWaitForStart() {
        prompter.run();
    }

    @Override
    public void onStartButtonPressed() {
        // Schedule the proper auto
        switch (selectedAuto) {
            case LEAVE_NEAR_LAUNCH:
                Leave_Near_Launch().invoke();
                return;
            case LEAVE_FAR_HP:
                Leave_Far_HP().invoke();
                return;
        }
    }

    @Override
    public void onUpdate() {
        if (turretOn){
            switch (selectedAlliance) {
                case RED:
                    Robot.turret.updateAimbot(Robot.drivetrain.follower.getPose(), true);
                    return;
                case BLUE:
                    Robot.turret.updateAimbot(Robot.drivetrain.follower.getPose(), false);
                    return;
            }
        }

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
        return new SequentialGroup(
                Commands.setFlywheelState(Robot, Turret.flywheelstate.ON),
                new ParallelGroup(
                        new WaitUntil(() -> Robot.turret.getFlywheelRPM() > 5900),
                        new Delay(5)
                ),
                Commands.setIntakeMode(Robot, Intake.Mode.SHOOTING),
                new Delay(2),
                Commands.setIntakeMode(Robot, Intake.Mode.INTAKING),
                new Delay(1),
                Commands.setIntakeMode(Robot, Intake.Mode.SHOOTING),
                new Delay(2),
                Commands.setTransferState(Robot, Intake.transferState.UP),
                new Delay(1),
                Commands.setTransferState(Robot, Intake.transferState.DOWN),
                Commands.setIntakeMode(Robot, Intake.Mode.OFF),
                Commands.setFlywheelState(Robot, Turret.flywheelstate.OFF),
                Commands.runPath(Robot.drivetrain.paths.start2_to_leave3, true, 0.6)
        );
    }

    /**
     * This auto starts at the near zone,
     * shoots it's preloads after 5 seconds,
     * and goes to the leave pose that is in the launch zone.
     */
    private Command Leave_Near_Launch() {
        turretOn = true;
        return new SequentialGroup(
                Commands.setFlywheelState(Robot, Turret.flywheelstate.ON),
                new ParallelGroup(
                        new WaitUntil(() -> Robot.turret.getFlywheelRPM() > 5900),
                        new Delay(5)
                ),
                Commands.setIntakeMode(Robot, Intake.Mode.SHOOTING),
                new Delay(2),
                Commands.setIntakeMode(Robot, Intake.Mode.INTAKING),
                new Delay(1),
                Commands.setIntakeMode(Robot, Intake.Mode.SHOOTING),
                new Delay(2),
                Commands.setTransferState(Robot, Intake.transferState.UP),
                new Delay(1),
                Commands.setTransferState(Robot, Intake.transferState.DOWN),
                Commands.setIntakeMode(Robot, Intake.Mode.OFF),
                Commands.setFlywheelState(Robot, Turret.flywheelstate.OFF),
                Commands.runPath(Robot.drivetrain.paths.start1_to_leave1, true, 0.6)
        );
    }
}
