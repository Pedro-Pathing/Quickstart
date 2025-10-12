package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ShooterSubsystem;

public class StopStateCommand extends InstantCommand {
    public StopStateCommand(ShooterSubsystem.StopState state) {
        super(
                () -> Robot.getInstance().shooterSubsystem.updateStopState(state)
        );
    }
}