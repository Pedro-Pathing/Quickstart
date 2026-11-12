package org.firstinspires.ftc.teamcode.commands.gate;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.gate.Gate;

public class GateClose extends InstantCommand {
    public GateClose(Gate gate) {
        super(gate::close, gate);
    }
}
