package org.firstinspires.ftc.teamcode.CommandBase.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.CommandBase.Subsystems.Intake;

public class IntakeCommand extends CommandBase {
    Intake intake;

    public IntakeCommand(Intake intake) {
        this.intake = intake;

    }

    @Override
    public void execute() {
        intake.startIntake();
    }
}
