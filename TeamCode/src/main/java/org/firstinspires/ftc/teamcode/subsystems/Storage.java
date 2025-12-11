package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;


public class Storage implements Subsystem {
    public static final Storage INSTANCE = new Storage();

    private MotorEx spin = new MotorEx("motorExp0")
            .brakeMode()
            .zeroed();

    private ControlSystem controlSystem = ControlSystem.builder()
            .posPid(0.005, 0, 0)
            .elevatorFF(0)
            .build();

    public Command toIntake1 = new RunToPosition(controlSystem, 0).requires(this);
    public Command toIntake2 = new RunToPosition(controlSystem, 500).requires(this);
    public Command toIntake3 = new RunToPosition(controlSystem, 1200).requires(this);

    @Override
    public void initialize() {
    }

    @Override
    public void periodic() {
        spin.setPower(controlSystem.calculate(spin.getState()));
    }

}
