package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.StopStateCommand;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ShooterSubsystem;

public class IntakeStopCommand extends SequentialCommandGroup {
    public
    IntakeStopCommand() {
        super(
                new IntakeStateCommand(IntakeSubsystem.IntakeState.STOP)
        );
    }
}
