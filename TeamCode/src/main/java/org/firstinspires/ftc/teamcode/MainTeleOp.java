package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.utils.Logger;

@TeleOp(name="MainTeleOp", group="TeleOp")
public class MainTeleOp extends NextFTCOpMode {
    {
        addComponents(
                BulkReadComponent.INSTANCE, // TODO: make actual MANUAL mode bulkreading (we don't need to also read the expansion hub every loop)
                BindingsComponent.INSTANCE,
                CommandManager.INSTANCE,
                new SubsystemComponent(
                        Storage.INSTANCE,
                        //Robot.INSTANCE,
                        Drive.INSTANCE,
                        Intake.INSTANCE,
                        Outtake.INSTANCE,
                        Transitions.INSTANCE
                ),
                new PedroComponent(Constants::createFollower)
        );
    }

    @Override public void onInit() {
    }
    @Override public void onWaitForStart() {
        ActiveOpMode.telemetry().update();
    }
    @Override public void onStartButtonPressed() {

        GamepadEx caimo = Gamepads.gamepad1();
        GamepadEx jeff = Gamepads.gamepad2();

        caimo.rightBumper()
                .whenBecomesTrue(() -> Intake.setIntakePowerCommand(1).schedule())
                .whenBecomesFalse(() -> Intake.setIntakePowerCommand(0).schedule());
        caimo.leftBumper()
                .whenBecomesTrue(() -> Intake.setIntakePowerCommand(-1).schedule())
                .whenBecomesFalse(() -> Intake.setIntakePowerCommand(0).schedule());
        jeff.x()
                .whenBecomesTrue(() -> {
                    Storage.setManualMode(true);
                    Storage.setManualPower(1);
                })
                .whenBecomesFalse(() -> {
                    Storage.setManualMode(true);
                    Storage.setManualPower(0);
                });
        jeff.a()
                .whenBecomesTrue(Storage::resetEncoder);
        jeff.rightBumper()
                .whenBecomesTrue(() -> Outtake.setOuttakePowerCommand(1).schedule())
                .whenBecomesFalse(() -> Outtake.setOuttakePowerCommand(0).schedule());
        jeff.leftBumper()
                .whenBecomesTrue(() -> Outtake.setOuttakePowerCommand(-1).schedule())
                .whenBecomesFalse(() -> Outtake.setOuttakePowerCommand(0).schedule());
        jeff.b()
                .whenBecomesTrue(() -> Transitions.setOuttakePositionCommand(Transitions.DOWN_POS).schedule())
                .whenBecomesFalse(() -> Transitions.setOuttakePositionCommand(Transitions.UP_POS).schedule());
        jeff.dpadDown()
                .whenBecomesTrue(() -> Storage.spinToNextIntakeIndex().schedule());
        jeff.dpadUp()
                .whenBecomesTrue(() -> Storage.spinToNextOuttakeIndex().schedule());
    }
    @Override public void onUpdate() {
        for (String cname : CommandManager.INSTANCE.snapshot()) {
            Logger.add("Commands", Logger.Level.DEBUG, cname);
        }
        Logger.update(Logger.Level.DEBUG);
    }

    @Override public void onStop() {
    }
}
