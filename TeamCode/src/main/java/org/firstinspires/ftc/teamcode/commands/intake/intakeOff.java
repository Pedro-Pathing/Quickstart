package org.firstinspires.ftc.teamcode.commands.intake;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;

public class intakeOff extends CommandBase {
    private final Intake intake;

    public intakeOff(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void initialize(){
        intake.setState(Intake.State.OFF);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
