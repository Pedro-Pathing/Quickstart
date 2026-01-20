package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.utils.Alliance;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.SubsystemGroup;
public class Robot extends SubsystemGroup {
    public static final Robot INSTANCE = new Robot();
    private static final double DELAY = 0.5;

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
            new WaitUntil(Outtake::reachedTargetVelocity),
            Transitions.on(),
            new Delay(DELAY),
            new WaitUntil(Outtake::reachedTargetVelocity),
            Storage.spinToNextOuttakeIndex(),
            new Delay(DELAY),
            new WaitUntil(Outtake::reachedTargetVelocity),
            Storage.spinToNextOuttakeIndex(),
            new Delay(DELAY),
            Transitions.off(),
            Outtake.off
    );

    public static SequentialGroup outtakeOne = new SequentialGroup(
            Outtake.on,
            Storage.spinToNextOuttakeIndex(),
            new WaitUntil(Outtake::reachedTargetVelocity),
            Transitions.on(),
            new Delay(DELAY),
            Transitions.off(),
            Outtake.off
    );

    public static SequentialGroup intakeAll = new SequentialGroup(

    );
}
