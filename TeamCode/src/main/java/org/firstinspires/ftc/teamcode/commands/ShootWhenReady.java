package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import java.util.function.BooleanSupplier;

public class ShootWhenReady extends SequentialCommandGroup {
    public static long TIMEOUT_MS = 1500;

    public ShootWhenReady(BooleanSupplier okToShoot) {
        addCommands(
                new WaitUntilCommand(okToShoot).withTimeout(TIMEOUT_MS),
                new TransferSequence()
        );
    }
}
