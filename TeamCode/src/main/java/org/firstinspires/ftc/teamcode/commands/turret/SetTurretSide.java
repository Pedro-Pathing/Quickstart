package org.firstinspires.ftc.teamcode.commands.turret;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.util.PosePersistency;

public class SetTurretSide extends InstantCommand {
    public SetTurretSide(Turret turret, Field.Alliance alliance) {
        super(() -> {
            turret.setAlliance(alliance);
            PosePersistency.saveAlliance(alliance);
        });
    }
}
