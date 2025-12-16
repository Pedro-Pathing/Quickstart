package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;

public class Storage implements Subsystem {
    public static final Storage INSTANCE = new Storage();

    private MotorEx spin = new MotorEx("motor2")
            .brakeMode()
            .zeroed();

    private ControlSystem controlSystem = ControlSystem.builder()
            .posPid(10, 0, 0)
            .elevatorFF(0)
            .build();

    public Command toIntake1 = new RunToPosition(controlSystem, 0).requires(this);
    public Command toIntake2 = new RunToPosition(controlSystem, 180).requires(this);
    public Command toIntake3 = new RunToPosition(controlSystem, 360).requires(this);
    public Command toOuttake1 = new RunToPosition(controlSystem, 1200).requires(this);
    public Command toOuttake2 = new RunToPosition(controlSystem, 1200).requires(this);
    public Command toOuttake3 = new RunToPosition(controlSystem, 1200).requires(this);

    public Command resetSpinMotor = new InstantCommand(() -> spin.zero());

    @Override
    public void initialize() {
        spin.zero();
    }

    @Override
    public void periodic() {
        //spin.setPower(controlSystem.calculate(spin.getState()));
        ActiveOpMode.telemetry().addData("Storage Position", spin.getCurrentPosition());
        ActiveOpMode.telemetry().update();
    }

}
