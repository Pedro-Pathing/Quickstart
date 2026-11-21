package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.gate.Gate;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

import java.util.function.BooleanSupplier;

public class ShootWhenReady extends SequentialCommandGroup {
    public static long TIMEOUT_MS = 1500;
    Intake intake;
    Gate gate;
    Turret turret;

    public ShootWhenReady(BooleanSupplier okToShoot, Intake intake, Gate gate, Turret turret) {
        addCommands(
                new WaitUntilCommand(okToShoot).withTimeout(TIMEOUT_MS),
                new TransferSequence(intake, gate, turret)



        );
    }
}
