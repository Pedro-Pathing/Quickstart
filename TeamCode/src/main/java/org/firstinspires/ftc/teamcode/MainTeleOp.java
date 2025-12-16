package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import org.firstinspires.ftc.teamcode.subsystems.*;

import static dev.nextftc.bindings.Bindings.button;

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
                        Outtake.INSTANCE
                )
                //new PedroComponent(Constants::createFollower)
        );
    }

    @Override public void onInit() {
        button(() -> gamepad2.a)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> Intake.setIntakePower(1))
                .whenBecomesFalse(() -> Intake.setIntakePower(0));

        button(() -> gamepad2.x)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Outtake.on)
                .whenBecomesFalse(Outtake.off);

        button(() -> gamepad2.b)
                .whenBecomesTrue(() -> Storage.INSTANCE.resetSpinMotor.schedule());

        button(() -> gamepad2.dpad_left)
                .whenBecomesTrue(() -> Storage.INSTANCE.toIntake1.schedule());
        button(() -> gamepad2.dpad_down)
                .whenBecomesTrue(() -> Storage.INSTANCE.toIntake2.schedule());
        button(() -> gamepad2.dpad_right)
                .whenBecomesTrue(() -> Storage.INSTANCE.toIntake3.schedule());
    }
    @Override public void onWaitForStart() {

    }
    @Override public void onStartButtonPressed() {

    }
    @Override public void onUpdate() {

    }
    @Override public void onStop() {

    }
}
