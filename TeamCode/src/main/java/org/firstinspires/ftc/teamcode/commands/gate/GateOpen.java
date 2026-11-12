package org.firstinspires.ftc.teamcode.commands.gate;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.gate.Gate;

public class GateOpen extends InstantCommand {
    public GateOpen(Gate gate) {
        super(gate::open, gate);
    }
}
