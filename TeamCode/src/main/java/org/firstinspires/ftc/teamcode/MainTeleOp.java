package org.firstinspires.ftc.teamcode;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class MainTeleOp extends NextFTCOpMode {
    {
        addComponents(
                BulkReadComponent.INSTANCE, // TODO: make actual MANUAL mode bulkreading (we don't need to also read the expansion hub every loop)
                BindingsComponent.INSTANCE,
                CommandManager.INSTANCE,
                new SubsystemComponent(Robot.INSTANCE),
                new PedroComponent(Constants::createFollower)
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
