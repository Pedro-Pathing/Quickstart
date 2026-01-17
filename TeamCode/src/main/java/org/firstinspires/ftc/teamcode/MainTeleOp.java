package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Storage;
import org.firstinspires.ftc.teamcode.subsystems.Transitions;
import org.firstinspires.ftc.teamcode.utils.Logger;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name="MainTeleOp", group="TeleOp")
public class MainTeleOp extends NextFTCOpMode {
    {
        addComponents(
                BulkReadComponent.INSTANCE, // TODO: make actual MANUAL mode bulkreading (we don't need to also read the expansion hub every loop)
                BindingsComponent.INSTANCE,
                CommandManager.INSTANCE,
                new SubsystemComponent(
                        Storage.INSTANCE,
                        Robot.INSTANCE,
                        Drive.INSTANCE,
                        Intake.INSTANCE,
                        Outtake.INSTANCE,
                        Transitions.INSTANCE
                )
        );
    }

    @Override public void onInit() {
    }
    @Override public void onWaitForStart() {
        Logger.update();
    }
    @Override public void onStartButtonPressed() {

        GamepadEx gamepad1 = Gamepads.gamepad1();
        GamepadEx gamepad2 = Gamepads.gamepad2();

    }
    @Override public void onUpdate() {
        for (String cname : CommandManager.INSTANCE.snapshot()) {
            Logger.add("Commands", cname);
        }
        Logger.update();
    }

    @Override public void onStop() {
    }
}
