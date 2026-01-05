package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import org.firstinspires.ftc.teamcode.subsystems.*;

@TeleOp(name="MainTeleOp", group="TeleOp")
public class MainTeleOp extends NextFTCOpMode {
    {
        addComponents(
                //BulkReadComponent.INSTANCE, // TODO: make actual MANUAL mode bulkreading (we don't need to also read the expansion hub every loop)
                BindingsComponent.INSTANCE,
                CommandManager.INSTANCE,
                new SubsystemComponent(
                        Storage.INSTANCE,
                        //Robot.INSTANCE,
                        //Drive.INSTANCE,
                        Intake.INSTANCE,
                        //Outtake.INSTANCE,
                        Transitions.INSTANCE
                )
                //new PedroComponent(Constants::createFollower)
        );
    }

    @Override public void onInit() {
    }
    @Override public void onWaitForStart() {
        ActiveOpMode.telemetry().update();
    }
    @Override public void onStartButtonPressed() {
        Gamepads.gamepad1().x()
                .whenBecomesTrue(() -> Storage.setManualPower(1))
                .whenBecomesFalse(() -> Storage.setManualPower(0));
        Gamepads.gamepad1().a()
                .whenBecomesTrue(Storage::resetEncoder);
        Gamepads.gamepad1().y()
                .whenBecomesTrue(() -> Intake.setIntakePower(1))
                .whenBecomesFalse(() -> Intake.setIntakePower(0));
        Gamepads.gamepad1().b()
                .whenBecomesTrue(() -> Transitions.setOuttakePosition(0.1))
                .whenBecomesFalse(() -> Transitions.setOuttakePosition(0.9));
        Gamepads.gamepad1().dpadDown()
                .whenBecomesTrue(() -> Storage.spinForwardTicks(180).schedule());
    }
    @Override public void onUpdate() {
        ActiveOpMode.telemetry().update();
    }
    @Override public void onStop() {
    }
}
