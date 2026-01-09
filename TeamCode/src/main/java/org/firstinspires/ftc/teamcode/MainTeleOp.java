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
import dev.nextftc.ftc.ActiveOpMode;
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
                //new PedroComponent(Constants::createFollower)
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

        jeff.back()
                .whenBecomesTrue(() -> {
                    Storage.resetEncoderCommand().schedule();
                });

        jeff.a()
                .whenBecomesTrue(() -> {
                    Storage.setManualModeCommand(true).schedule();
                    Storage.setManualPowerCommand(0.025).schedule();
                })
                .whenBecomesFalse(() -> {
                    Storage.setManualModeCommand(true).schedule();
                    Storage.setManualPowerCommand(0).schedule();
                });

        jeff.y()
                .whenBecomesTrue(() -> {
                    Storage.setManualModeCommand(true).schedule();
                    Storage.setManualPowerCommand(0.15).schedule();
                })
                .whenBecomesFalse(() -> {
                    Storage.setManualModeCommand(true).schedule();
                    Storage.setManualPowerCommand(0).schedule();
                });


        caimo.a()
                .whenBecomesTrue(() -> {
                    Outtake.on.schedule();
                })
                .whenBecomesFalse(() -> {

                });

        jeff.b()
                .whenBecomesTrue(() -> {
                    Storage.setManualModeCommand(true).schedule();
                    Storage.setManualPowerCommand(0.33).schedule();
                })
                .whenBecomesFalse(() -> {
                    Storage.setManualModeCommand(true).schedule();
                    Storage.setManualPowerCommand(0).schedule();
                });

        caimo.back()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> Drive.setHeadingLock(true))
                .whenBecomesFalse(() -> Drive.setHeadingLock(false));

        jeff.x()
                .whenBecomesTrue(() -> Transitions.setOuttakePositionCommand(Transitions.DOWN_POS).schedule())
                .whenBecomesFalse(() -> Transitions.setOuttakePositionCommand(Transitions.UP_POS).schedule());

        jeff.rightBumper()
                .whenBecomesTrue(() -> Outtake.setOuttakePowerCommand(1).schedule())
                .whenBecomesFalse(() -> Outtake.setOuttakePowerCommand(0).schedule());
        jeff.leftBumper()
                .whenBecomesTrue(() -> Outtake.setOuttakePowerCommand(0.8).schedule())
                .whenBecomesFalse(() -> Outtake.setOuttakePowerCommand(0).schedule());
        caimo.rightBumper()
                .whenBecomesTrue(() -> Intake.setIntakePowerCommand(1).schedule())
                .whenBecomesFalse(() -> Intake.setIntakePowerCommand(0).schedule());
        caimo.leftBumper()
                .whenBecomesTrue(() -> Intake.setIntakePowerCommand(-1).schedule())
                .whenBecomesFalse(() -> Intake.setIntakePowerCommand(0).schedule());

        jeff.dpadDown()
                .whenBecomesTrue(() -> Storage.spinToNextIntakeIndex().schedule());
        jeff.dpadUp()
                .whenBecomesTrue(() -> Storage.spinToNextOuttakeIndex().schedule());
        caimo.dpadDown()
                .whenBecomesTrue(() -> Storage.spinToNextIntakeIndex().schedule());
        caimo.dpadUp()
                .whenBecomesTrue(() -> Storage.spinToNextOuttakeIndex().schedule());

        caimo.dpadLeft()
                .whenBecomesTrue(() -> Robot.intakeAll.schedule());
        caimo.dpadRight()
                .whenBecomesTrue(() -> Robot.outtakeAll.schedule());
        jeff.dpadLeft()
                .whenBecomesTrue(() -> Robot.intakeAll.schedule());
        jeff.dpadRight()
                .whenBecomesTrue(() -> Robot.outtakeAll.schedule());
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
