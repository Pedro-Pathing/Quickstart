package org.firstinspires.ftc.teamcode.commands.IntakeFlower;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.intakeFlower.IntakeFlower;

public class IntakeFlowerDown extends CommandBase {
    public final IntakeFlower intakeFlower;

    public IntakeFlowerDown(IntakeFlower intakeFlower) {
        this.intakeFlower = intakeFlower;
    }

    @Override
    public void initialize(){intakeFlower.intakeFlowerOn();}

    @Override
    public boolean isFinished(){return true;}

}
