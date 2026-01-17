package org.firstinspires.ftc.teamcode.LOADCode.Main_.Auto_;

import static org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass.selectedAlliance;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.skeletonarmy.marrow.prompts.OptionPrompt;
import com.skeletonarmy.marrow.prompts.Prompter;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Intake;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Turret;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Commands;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Drivetrain_.MecanumDrivetrainClass;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Drivetrain_.Pedro_Paths;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.LoadHardwareClass;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Auto_Main_", group = "Main", preselectTeleOp="Teleop_Main_")
public class Auto_Main_ extends NextFTCOpMode {

    // Variable to store the selected auto program
    Auto selectedAuto = null;
    // Create the prompter object for selecting Alliance and Auto
    Prompter prompter = null;
    // Create a new instance of our Robot class
    LoadHardwareClass Robot = new LoadHardwareClass(this);
    // Create a Paths object for accessing modular auto paths
    Pedro_Paths paths = new Pedro_Paths();
    // Create a Commands object for auto creation
    Commands Commands = new Commands(Robot);

    // Auto parameter variables
    private boolean turretOn = true;

    @SuppressWarnings("unused")
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
                        new Near_12Ball(),
                        new Near_9Ball(),
                        new Leave_Far_Generic()
                        //new test_Auto(),
                        //new Complex_Test_Auto()
                ));
        prompter.onComplete(() -> {
                    selectedAlliance = prompter.get("alliance");
                    selectedAuto = prompter.get("auto");
                    telemetry.addData("Selection", "Complete");
                    telemetry.addData("Alliance", selectedAlliance.toString());
                    telemetry.addData("Auto", selectedAuto);
                    telemetry.update();
                    // Build paths
                    paths.buildPaths(selectedAlliance, follower());
                    // Initialize all hardware of the robot
                    Robot.init(paths.autoMirror(selectedAuto.getStartPose(), selectedAlliance), follower());
                    while (opModeInInit() && Robot.turret.zeroTurret()){
                        sleep(0);
                    }
        });
    }

    @Override
    public void onWaitForStart() {
        prompter.run();
    }

    @Override
    public void onStartButtonPressed() {
        // Schedule the proper auto
        selectedAuto.runAuto();
        turretOn = selectedAuto.getTurretEnabled();

        // Indicate that initialization is done
        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void onUpdate() {
        telemetry.addData("Running Auto", selectedAuto.toString());
        telemetry.addLine();
        if (turretOn){
            telemetry.addData("Aimbot Target", selectedAlliance);
        }else{
            telemetry.addLine("Aimbot Off");
        }
        Robot.turret.updateAimbot(turretOn, true, 0);
        Robot.turret.updateFlywheel();

        MecanumDrivetrainClass.robotPose = Robot.drivetrain.follower.getPose();

        telemetry.addLine();
        telemetry.addData("FarStart", paths.farStart);
        telemetry.addData("FarShoot", paths.noTurretFarShoot);
        telemetry.update();
    }

    @Override
    public void onStop(){
        Robot.drivetrain.follower.holdPoint(Robot.drivetrain.follower.getPose());
        MecanumDrivetrainClass.robotPose = Robot.drivetrain.follower.getPose();
    }

    /**
     * This class serves as a template for all auto programs. </br>
     * The methods runAuto() and ToString() must be overridden for each auto.
     */
    abstract static class Auto{
//        /**
//         * This constructor must be called from the child class using <code>super()</code>
//         * @param startingPose Indicates the starting pose of the robot
//         * @param runTurret Indicates whether to run the turret auto aim functions
//         */
//        Auto(Pose startingPose, Boolean runTurret){
//            turretOn = runTurret;
//            startPose = startingPose;
//        }
//        Auto(Pose startingPose){
//            turretOn = true;
//            startPose = startingPose;
//        }

        abstract Pose getStartPose();
        abstract boolean getTurretEnabled();

        /** Override this to schedule the auto command*/
        abstract void runAuto();
        /** Override this to rename the auto*/
        @NonNull
        @Override
        public abstract String toString();
    }

    private class Leave_Far_Generic extends Auto{
        public Pose startPose = paths.farStart;
        public boolean turretEnabled = false;

        @Override
        public Pose getStartPose(){
            return startPose;
        }
        @Override
        public boolean getTurretEnabled(){
            return turretEnabled;
        }

        @Override
        public void runAuto(){
            new SequentialGroup(
                    Commands.runPath(paths.farStart_to_NoTurret_FarShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.setIntakeMode(Intake.intakeMode.INTAKING),
                    Commands.runPath(paths.farShoot_noTurret_to_farPreload, true, 1),
                    Commands.setIntakeMode(Intake.intakeMode.OFF)
            ).schedule();
        }

        @NonNull
        @Override
        public String toString(){return "Far Zone No Turret Generic";}
    }

    private class Near_9Ball extends Auto{
        public Pose startPose = paths.nearStart;
        public boolean turretEnabled = true;

        @Override
        public Pose getStartPose(){
            return startPose;
        }
        @Override
        public boolean getTurretEnabled(){
            return turretEnabled;
        }

        @Override
        public void runAuto(){
            new SequentialGroup(
                    Commands.setIntakeMode(Intake.intakeMode.INTAKING),
                    new InstantCommand(Commands.setFlywheelState(Turret.flywheelState.ON)),
                    Commands.runPath(paths.nearStart_to_midShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.setFlywheelState(Turret.flywheelState.ON),
                    Commands.setIntakeMode(Intake.intakeMode.INTAKING),
                    Commands.runPath(paths.midShoot_to_nearPreload, true, 1),
                    Commands.runPath(paths.nearPreload_to_midShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.setFlywheelState(Turret.flywheelState.ON),
                    Commands.setIntakeMode(Intake.intakeMode.INTAKING),
                    Commands.runPath(paths.midShoot_to_midPreload, true, 1),
                    Commands.runPath(paths.midPreload_to_midShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.runPath(paths.midShoot_to_nearLeave, true, 1)
            ).schedule();
        }

        @NonNull
        @Override
        public String toString(){return "Near Zone 9 Ball Auto";}
    }

    private class Near_12Ball extends Auto{
        public Pose startPose = paths.nearStart;
        public boolean turretEnabled = true;

        @Override
        public Pose getStartPose(){
            return startPose;
        }
        @Override
        public boolean getTurretEnabled(){
            return turretEnabled;
        }

        @Override
        public void runAuto(){
            new SequentialGroup(
                    Commands.setIntakeMode(Intake.intakeMode.INTAKING),
                    new InstantCommand(Commands.setFlywheelState(Turret.flywheelState.ON)),
                    Commands.runPath(paths.nearStart_to_midShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.setFlywheelState(Turret.flywheelState.ON),
                    Commands.setIntakeMode(Intake.intakeMode.INTAKING),
                    Commands.runPath(paths.midShoot_to_nearPreload, true, 1),
                    Commands.runPath(paths.nearPreload_to_midShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.setFlywheelState(Turret.flywheelState.ON),
                    Commands.setIntakeMode(Intake.intakeMode.INTAKING),
                    Commands.runPath(paths.midShoot_to_midPreload, true, 1),
                    Commands.runPath(paths.midPreload_to_midShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.setFlywheelState(Turret.flywheelState.ON),
                    Commands.setIntakeMode(Intake.intakeMode.INTAKING),
                    Commands.runPath(paths.midShoot_to_farPreload, true, 1),
                    Commands.runPath(paths.farPreload_to_midShoot, true, 1),
                    Commands.shootBalls(),
                    Commands.runPath(paths.midShoot_to_nearLeave, true, 1)
            ).schedule();
        }

        @NonNull
        @Override
        public String toString(){return "Near Zone 12 Ball Auto";}
    }

    /**
     * This auto starts at the far zone
     */
    private class Complex_Test_Auto extends Auto{
        public Pose startPose = paths.farStart;
        public boolean turretEnabled = true;

        @Override
        public Pose getStartPose(){
            return startPose;
        }
        @Override
        public boolean getTurretEnabled(){
            return turretEnabled;
        }

        @Override
        void runAuto() {
            new SequentialGroup(
                    new Delay(1),
                    //Commands.shootBalls(),
                    Commands.setIntakeMode(Intake.intakeMode.INTAKING),
                    Commands.runPath(paths.farStart_to_farPreload, true, 1),
                    Commands.setIntakeMode(Intake.intakeMode.OFF),
                    Commands.runPath(paths.farPreload_to_farShoot, true, 1),
                    //Commands.shootBalls(),
                    Commands.setIntakeMode(Intake.intakeMode.INTAKING),
                    Commands.runPath(paths.farStart_to_midPreload, true, 1),
                    Commands.setIntakeMode(Intake.intakeMode.OFF),
                    Commands.runPath(paths.midPreload_to_farShoot, true, 1),
                    //Commands.shootBalls(),
                    Commands.runPath(paths.farShoot_to_farLeave, true, 1)
            ).schedule();
        }

        @NonNull
        @Override
        public String toString() {
            return "Complex Test Auto";
        }
    }

    private class test_Auto extends Auto{
        public Pose startPose = paths.farStart;
        public boolean turretEnabled = false;

        @Override
        public Pose getStartPose(){
            return startPose;
        }
        @Override
        public boolean getTurretEnabled(){
            return turretEnabled;
        }

        @Override
        public void runAuto(){
            double tempSpeed = 1;
            new SequentialGroup(
                    Commands.runPath(paths.farStart_to_farPreload,true,tempSpeed),
                    Commands.runPath(paths.farPreload_to_farShoot,true,tempSpeed),
                    Commands.runPath(paths.farShoot_to_midPreload, true, tempSpeed),
                    Commands.runPath(paths.midPreload_to_midShoot, true, tempSpeed),
                    Commands.runPath(paths.midShoot_to_nearPreload, true, tempSpeed),
                    Commands.runPath(paths.nearPreload_to_nearShoot, true, tempSpeed)
            ).schedule();
        }

        @NonNull
        @Override
        public String toString(){return "Test Auto";}
    }
}
