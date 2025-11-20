package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name="MainTeleOp", group="TeleOp")
public class MainTeleOp extends NextFTCOpMode {
    {
        addComponents(
                BulkReadComponent.INSTANCE, // TODO: make actual MANUAL mode bulkreading (we don't need to also read the expansion hub every loop)
                BindingsComponent.INSTANCE,
                CommandManager.INSTANCE,
                new SubsystemComponent(
                        Robot.INSTANCE,
                        Drive.INSTANCE,
                        Intake.INSTANCE,
                        Outtake.INSTANCE
                )
                //new PedroComponent(Constants::createFollower)
        );
    }

    @Override public void onInit() {

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
