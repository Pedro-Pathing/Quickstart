package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.SubsystemGroup;
public class Robot extends SubsystemGroup {
    public static final Robot INSTANCE = new Robot();

    public enum Alliance {
        RED,
        BLUE
    }

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
            new InstantCommand(Outtake.on),
            new InstantCommand(Storage.spinToNextOuttakeIndex()),
            new Delay(0.5),
            new InstantCommand(Transitions.setOuttakePositionCommand(Transitions.UP_POS)),
            new Delay(0.1),
            new InstantCommand(Transitions.setOuttakePositionCommand(Transitions.DOWN_POS)),
            new Delay(1),
            new InstantCommand(Storage.spinToNextOuttakeIndex()),
            new Delay(0.5),
            new InstantCommand(Transitions.setOuttakePositionCommand(Transitions.UP_POS)),
            new Delay(0.1),
            new InstantCommand(Transitions.setOuttakePositionCommand(Transitions.DOWN_POS)),
            new Delay(1),
            new InstantCommand(Storage.spinToNextOuttakeIndex()),
            new Delay(0.5),
            new InstantCommand(Transitions.setOuttakePositionCommand(Transitions.UP_POS)),
            new Delay(0.1),
            new InstantCommand(Transitions.setOuttakePositionCommand(Transitions.DOWN_POS)),
            new Delay(1),
            new InstantCommand(Outtake.setRunDownCommand(true))
    );

    public static SequentialGroup intakeAll = new SequentialGroup(
            new InstantCommand(Storage.spinToNextIntakeIndex()),
            new InstantCommand(Intake.setIntakePowerCommand(1)),
            new InstantCommand(Storage.spinToNextIntakeIndex()),
            new Delay(0.25),
            new InstantCommand(Storage.spinToNextIntakeIndex()),
            new Delay(0.25),
            new InstantCommand(Intake.setIntakePowerCommand(-1)),
            new Delay(0.5),
            new InstantCommand(Intake.setIntakePowerCommand(0)));
}
