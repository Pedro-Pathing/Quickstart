package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;

public class IntakeOutCommand extends SequentialCommandGroup {
    public
    IntakeOutCommand() {
        super(
                new IntakeStateCommand(IntakeSubsystem.IntakeState.OUT)
        );
    }
}
