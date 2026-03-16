package org.firstinspires.ftc.teamcode.OpModes.Teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandBase.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Global.Constants;
import org.firstinspires.ftc.teamcode.Global.Robot;

@TeleOp
public class TEST extends CommandOpMode {
    Robot robot = Robot.getInstance();
    GamepadEx driver;

    MecanumDrive mecanumDrive;

    @Override
    public void initialize() {
        robot.init(hardwareMap);
        driver = new GamepadEx(gamepad1);

        driver.getGamepadButton(GamepadKeys.Button.A)
                .whileHeld(new InstantCommand(
                        () -> robot.intake.startIntake()))
                .whenReleased(new InstantCommand(
                        () -> robot.intake.startOuttake()
                ));

        driver.getGamepadButton(GamepadKeys.Button.START)
                .whenReleased(new InstantCommand(
                        () -> robot.drive.updateBotHeading()
                ));

        Trigger rightTrigger = new Trigger(
                () -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > Constants.GamePad.TRIGGER_THRESHOLD);
        Trigger leftTrigger = new Trigger(
                () -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > Constants.GamePad.TRIGGER_THRESHOLD);


        rightTrigger.whileActiveContinuous(new InstantCommand(
                () -> robot.intake.startIntake()
        ));

    }

    @Override
    public void run() {
        robot.drive.driveFieldCentric(
                -driver.getLeftY(),
                driver.getLeftX(),
                driver.getRightX(),
                robot.drive.getBotHeading()
        );

        telemetry.addData("Test: ", robot.imu.getAngularOrientation().firstAngle);
        telemetry.update();

        super.run();
    }

}
