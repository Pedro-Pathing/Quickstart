package org.firstinspires.ftc.teamcode.LOADCode.Main_.Auto_;

import static org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass.selectedAlliance;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Commands;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass;

@Autonomous(name = "Auto_Main_", group = "Main", preselectTeleOp="Teleop_Main_")
public class Auto_Main_ extends OpMode {

    // Define other PedroPathing constants
    private final Pose startPose = new Pose(87, 8.8, Math.toRadians(90)); // Start Pose of our robot.

    private enum Auto {
        test,
        LEAVE_FAR_HP
    }

    private int autoState = 0;

    private Auto selectedAuto = null;

    // Create the prompter object for selecting Alliance and Auto
    Prompter prompter = null;

    // Create a new instance of our Robot class
    LoadHardwareClass Robot = new LoadHardwareClass(this);

    @Override
    public void init() {
        // Initialize all hardware of the robot
        Robot.init(startPose);

        prompter = new Prompter(this);
        prompter.prompt("alliance",
                new OptionPrompt<>("Select Alliance",
                        LoadHardwareClass.Alliance.RED,
                        LoadHardwareClass.Alliance.BLUE
                ));
        prompter.prompt("auto",
                new OptionPrompt<>("Select Auto",
                        Auto.test,
                        Auto.LEAVE_FAR_HP
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
    public void init_loop() {
        prompter.run();
    }

    @Override
    public void start() {
        // Initialize all hardware of the robot
        Robot.init(startPose);
    }

    @Override
    public void loop() {
        switch (selectedAuto) {
            case test:
                test();
                return;
            case LEAVE_FAR_HP:
                Leave_Far_HP();
                return;
        }
        switch (selectedAlliance) {
            case RED:
                Robot.turret.updateAimbot(Robot.drivetrain.follower.getPose(), true);
                return;
            case BLUE:
                Robot.turret.updateAimbot(Robot.drivetrain.follower.getPose(), false);
                return;
        }

        telemetry.addData("selectedAuto", selectedAuto);
        telemetry.addData("follower", Robot.drivetrain.follower.getCurrentPath());
        telemetry.update();
    }

    /**
     * Waits for the current stage of the auto to be done and then runs the next stage.
     */
    private void waitForPathCompletion(){
        if (Robot.drivetrain.pathComplete()){autoState++;}
    }

    /**
     * This auto starts at the far zone,
     * shoots it's preloads after 5 seconds,
     * and goes to the human player leave zone.
     */
    private void Leave_Far_HP() {
        //Commands.setFlywheelState(Robot, Turret.flywheelstate.ON).schedule();
//        new ParallelGroup(
//                new WaitUntil(() -> Robot.turret.getFlywheelRPM() > 5900),
//                new Delay(5)
//        );
        Commands.runPath(Robot, Robot.drivetrain.paths.start2_to_leave3, true).schedule();
    }

    private void test() {
        Robot.drivetrain.runPath(Robot.drivetrain.paths.start2_to_leave3, true);
    }
}
