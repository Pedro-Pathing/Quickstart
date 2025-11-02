package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ShooterStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.StopStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;

public class ArtifactShootCommand extends SequentialCommandGroup {
    public
    ArtifactShootCommand() {
        super(
                new StopStateCommand(ShooterSubsystem.StopState.READY),
                new IntakeStateCommand(IntakeSubsystem.IntakeState.IN),
                new WaitCommand(250),
                new StopStateCommand(ShooterSubsystem.StopState.STOP)
//                new IntakeStateCommand(IntakeSubsystem.IntakeState.STOP)



        );
    }
}
