package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ShooterSubsystem;

public class ShooterStateCommand extends InstantCommand {
    public ShooterStateCommand(ShooterSubsystem.ShootState state) {
        super(
                () -> Robot.getInstance().shooterSubsystem.updateShootState(state)
        );
    }
}