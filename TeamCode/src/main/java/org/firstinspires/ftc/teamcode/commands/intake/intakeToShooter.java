package org.firstinspires.ftc.teamcode.commands.intake;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;

public class intakeToShooter extends CommandBase {
    private final Intake intake;

    public intakeToShooter(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void initialize(){
        intake.setState(Intake.State.FEED);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
