package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Storage.spinToNextOuttakeIndex;

import org.firstinspires.ftc.teamcode.utils.Logger;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.SubsystemGroup;
import dev.nextftc.ftc.ActiveOpMode;

public class Robot extends SubsystemGroup {
    public static final Robot INSTANCE = new Robot();

    private Robot() {
        super(
                Intake.INSTANCE,
                Transitions.INSTANCE,
                Storage.INSTANCE,
                Outtake.INSTANCE
        );
    }

    @Override
    public void initialize() {
    }

    @Override
    public void periodic() {
    }

    public static SequentialGroup outtakeAll = new SequentialGroup(
            Outtake.on,
            Storage.spinToNextOuttakeIndex(),
            Transitions.setOuttakePositionCommand(Transitions.UP_POS),
            Transitions.setOuttakePositionCommand(Transitions.DOWN_POS),
            new Delay(0.1),
            Storage.spinToNextOuttakeIndex(),
            Transitions.setOuttakePositionCommand(Transitions.UP_POS),
            Transitions.setOuttakePositionCommand(Transitions.DOWN_POS),
            new Delay(0.1),
            Storage.spinToNextOuttakeIndex(),
            Transitions.setOuttakePositionCommand(Transitions.UP_POS),
            Transitions.setOuttakePositionCommand(Transitions.DOWN_POS),
            new Delay(0.1),
            Outtake.off
    );

    public static SequentialGroup intakeAll = new SequentialGroup(
            Storage.spinToNextIntakeIndex(),
            Intake.setIntakePowerCommand(1),
            Storage.spinToNextIntakeIndex(),
            new Delay(0.25),
            Storage.spinToNextIntakeIndex(),
            new Delay(0.25),
            Intake.setIntakePowerCommand(-1),
            new Delay(0.25),
            Intake.setIntakePowerCommand(0)
            );
}
