package org.firstinspires.ftc.teamcode.LOADCode.Main_.Auto_;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass;

public class Auto_Main_ extends OpMode {

    // Define other PedroPathing constants
    private final Pose startPose = new Pose(87, 8.8, Math.toRadians(90)); // Start Pose of our robot.

    private enum Auto {
        LOAD_ROBOTICS_A,
        LOAD_ROBOTICS_B,
        MOE_A,
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
                        Auto.LOAD_ROBOTICS_A,
                        Auto.MOE_A,
                        Auto.LOAD_ROBOTICS_B
                ));
        prompter.onComplete(() -> {
                    LoadHardwareClass.selectedAlliance = prompter.get("alliance");
                    selectedAuto = prompter.get("auto");
                    telemetry.addData("Selection", "Complete");
                }
        );
    }

    @Override
    public void loop() {
        switch (selectedAuto) {
            case LOAD_ROBOTICS_A:
                return;
            case LOAD_ROBOTICS_B:
                return;
            default:
                TEMPLATE_A();
        }
    }

    /**
     * Waits for the current stage of the auto to be done and then runs the next stage.
     */
    private void waitForPathCompletion(){
        if (Robot.drivetrain.pathComplete()){autoState++;}
    }

     /**
     * Template auto
     */
    private void TEMPLATE_A() {
        switch (autoState){
            case 0:
                Robot.drivetrain.runPath(Robot.drivetrain.paths.startPose1_to_preload1, true);
                waitForPathCompletion();
            case 1:
                Robot.drivetrain.runPath(Robot.drivetrain.paths.startPose1_to_preload1, true);
                waitForPathCompletion();
            case 2:
                Robot.drivetrain.runPath(Robot.drivetrain.paths.startPose1_to_preload1, true);
                waitForPathCompletion();
        }
    }
}
