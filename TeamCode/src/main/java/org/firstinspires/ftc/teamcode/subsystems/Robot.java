package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
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
            Outtake.setOuttakePowerCommand(1),
            new Delay(0.1),
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
            Outtake.setOuttakePowerCommand(0)
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
