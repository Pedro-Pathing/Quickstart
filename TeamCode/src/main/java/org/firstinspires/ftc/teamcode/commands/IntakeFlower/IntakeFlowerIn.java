package org.firstinspires.ftc.teamcode.commands.IntakeFlower;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.intake.intakeOn;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intakeFlower.IntakeFlower;

public class IntakeFlowerIn extends SequentialCommandGroup {

    public IntakeFlowerIn(Intake intake, IntakeFlower intakeFlower){
        addRequirements(intake);
        addCommands(
                new IntakeFlowerDown(intakeFlower),
                new WaitCommand(500),
                new intakeOn(intake),
                new WaitCommand(1000)
                );
        addRequirements(intakeFlower);
    }
}
