package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.pedropathing.follower.Follower;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;


import org.firstinspires.ftc.teamcode.commands.advancedcommand.ArtifactInCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.ArtifactShootCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.RobotData;

import org.firstinspires.ftc.teamcode.pedroPathing.PedroPathingConstants;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.Supplier;


@TeleOp (name = "Solo")
public class TeleOp_Solo extends CommandOpMode {

    private final Robot robot = Robot.getInstance();
    private final RobotData data = Robot.getInstance().data;
    private GamepadEx g1;

    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private TelemetryManager telemetryM;

    boolean teleOpEnabled = false;

    boolean grabConfirmed = false;

    double fieldCentricOffset;

    boolean lastLeftTrigger;
    boolean lastRightTrigger;

    boolean lastA;
    boolean lastB;
    boolean lastX;
    boolean lastY;

    boolean lastLeftBumper;
    boolean lastRightBumper;

    boolean lastDpadUp;
    boolean lastDpadDown;
    boolean lastDpadLeft;
    boolean lastDpadRight;

    boolean lastRightStickButton;
    boolean lastLeftStickbutton;

    boolean lastPS;
    boolean lastStart;
    boolean lastBack;


    double lastIntakeSpeed;
    double lastIntakeDistance;


    @Override
    public void initialize() {

        g1 = new GamepadEx(gamepad1);

        Globals.IS_AUTO = false;

        follower = PedroPathingConstants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();


    }

    @Override
    public void run() {

        if (teleOpEnabled) {

            CommandScheduler.getInstance().run();

            robot.periodic();

            robot.updateData();
            robot.write();

            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false // Robot Centric
            );

        }

        boolean liftChangeJoystickUp = gamepad1.right_stick_y < -0.8;
        boolean liftChangeJoystickDown = gamepad1.right_stick_y > 0.8;


        boolean a = g1.getButton(GamepadKeys.Button.A);
        boolean b = g1.getButton(GamepadKeys.Button.B);
        boolean x = g1.getButton(GamepadKeys.Button.X);
        boolean y = g1.getButton(GamepadKeys.Button.Y);
        boolean leftBumper = g1.getButton(GamepadKeys.Button.LEFT_BUMPER);
        boolean rightBumper = g1.getButton(GamepadKeys.Button.RIGHT_BUMPER);
        boolean dpadUp = g1.getButton(GamepadKeys.Button.DPAD_UP);
        boolean dpadDown = g1.getButton(GamepadKeys.Button.DPAD_DOWN);
        boolean dpadLeft = g1.getButton(GamepadKeys.Button.DPAD_LEFT);
        boolean dpadRight = g1.getButton(GamepadKeys.Button.DPAD_RIGHT);
        boolean rightStickButton = g1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON);
        boolean leftStickButton = g1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON);
        boolean ps = gamepad1.ps;
        boolean start = g1.getButton(GamepadKeys.Button.START);
        boolean back = g1.getButton(GamepadKeys.Button.BACK);



        if (!lastX && x) {
            Constants.robotCentric = !Constants.robotCentric;
            gamepad1.rumble(500);
            if (Constants.robotCentric) gamepad1.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            else gamepad1.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        }

        if (!lastStart && start) {
            teleOpEnabled = true;
            gamepad1.rumble(2000);
        }





        lastA = a;
        lastB = b;
        lastX = x;
        lastY = y;
        lastLeftBumper = leftBumper;
        lastRightBumper = rightBumper;
        lastDpadUp = dpadUp;
        lastDpadDown = dpadDown;
        lastDpadLeft = dpadLeft;
        lastDpadRight = dpadRight;
        lastRightStickButton = rightStickButton;
        lastLeftStickbutton = leftStickButton;
        lastPS = ps;
        lastStart = start;

        boolean leftTrigger = gamepad1.left_trigger > .5;
        boolean rightTrigger = gamepad1.right_trigger > .5;

        if (leftTrigger && !lastLeftTrigger) {
            CommandScheduler.getInstance().schedule(new ArtifactShootCommand());
        }

        if (leftTrigger && !lastLeftTrigger) {
            CommandScheduler.getInstance().schedule(new ArtifactInCommand());
        }

        if(leftBumper && !lastLeftBumper) {
            CommandScheduler.getInstance().schedule((new IntakeStopCommand()));
        }

        lastLeftTrigger = leftTrigger;
        lastRightTrigger = rightTrigger;


        if (gamepad1.touchpad) {
            robot.follower.setPose(new Pose());
            gamepad1.rumble(500);
            gamepad1.setLedColor(0, 1, 0, 1000);

        }


    }

    private void scheduleCommand(boolean lastPress, boolean currPress, Command command) {
        if (currPress && !lastPress) {
            CommandScheduler.getInstance().schedule(command);
        }
    }
}
