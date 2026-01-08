package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.utils.Logger;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.MotorEx;

public class Outtake implements Subsystem {

    public static final Outtake INSTANCE = new Outtake();

    public static double outtakePower = 0;


    private static final MotorEx outtake = new MotorEx("motorExp3");
    private static final CRServoEx outtakeServo = new CRServoEx("servo4");

    private static final ControlSystem controller = ControlSystem.builder()
            .velPid(0.005, 0, 0)
            .basicFF(0.01, 0.02, 0.03)
            .build();

    public static Command off = new RunToVelocity(controller, 0.0).requires(INSTANCE).named("FlywheelOff");
    public static Command on = new RunToVelocity(controller, 500.0).requires(INSTANCE).named("FlywheelOn");

    public static void setOuttakePower(double newPower) {
        outtakePower = newPower;
    }

    @Override
    public void periodic() {

        outtake.setPower(outtakePower);
        Logger.add("Outtake", Logger.Level.DEBUG, "power: " + outtakePower);
        //motor.setPower(controller.calculate(motor.getState()))
        ;
    }
}
