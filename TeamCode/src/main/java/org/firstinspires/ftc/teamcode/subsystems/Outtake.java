package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.MotorEx;

public class Outtake implements Subsystem {

    public static final Outtake INSTANCE = new Outtake();

    private static final MotorEx motor = new MotorEx("motorExp1");
    private static final CRServoEx outtakeServo = new CRServoEx("servo4");

    private static final ControlSystem controller = ControlSystem.builder()
            .velPid(0.005, 0, 0)
            .basicFF(0.01, 0.02, 0.03)
            .build();

    public static Command off = new RunToVelocity(controller, 0.0).requires(INSTANCE).named("FlywheelOff");
    public static Command on = new RunToVelocity(controller, 500.0).requires(INSTANCE).named("FlywheelOn");

    @Override
    public void periodic() {
        motor.setPower(controller.calculate(motor.getState()));
    }
}
