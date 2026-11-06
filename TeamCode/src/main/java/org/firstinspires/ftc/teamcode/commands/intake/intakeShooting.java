package org.firstinspires.ftc.teamcode.commands.intake;

import com.seattlesolvers.solverslib.command.ScheduleCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

public class intakeShooting extends SequentialCommandGroup {
    public intakeShooting(Intake intake, Shooter shooter){
        addCommands(
                new intakeOff(intake),
                new WaitCommand(200),
                new intakeToShooter(intake),
                new WaitCommand(200),

                new intakeToShooter(intake),
                new WaitCommand(300),
                new intakeOff(intake)

        );
        addRequirements(intake);

    }


}
