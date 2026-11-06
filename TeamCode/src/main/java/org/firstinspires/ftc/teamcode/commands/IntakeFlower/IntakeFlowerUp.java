package org.firstinspires.ftc.teamcode.commands.IntakeFlower;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.intakeFlower.IntakeFlower;

public class IntakeFlowerUp extends CommandBase {

    public final IntakeFlower intakeFlower;

    public IntakeFlowerUp(IntakeFlower intakeFlower) {
        this.intakeFlower = intakeFlower;
    }

    @Override
    public void initialize(){intakeFlower.intakeFlowerOff();}

    @Override
    public boolean isFinished(){return true;}
}

