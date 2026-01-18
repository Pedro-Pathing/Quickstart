package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.conditionals.IfElseCommand;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.SubsystemGroup;
public class Robot extends SubsystemGroup {
    public static final Robot INSTANCE = new Robot();

    public static Robot.Alliance getCurrentAlliance(){
        return currentAlliance;
    }

    public static Robot.Alliance currentAlliance = Robot.Alliance.BLUE;

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
            new IfElseCommand(
                    Outtake::reachedTargetVelocity,
                    Outtake.on
    ));

    public static SequentialGroup intakeAll = new SequentialGroup(

    );
}
