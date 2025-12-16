package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Intake implements Subsystem {

    public static final Intake INSTANCE = new Intake();
    public static double intakePower = 0;

    private MotorEx intake = new MotorEx("motor0")
            .brakeMode()
            .zeroed();

    @Override
    public void initialize() {
    }

    @Override
    public void periodic() {
        intake.setPower(intakePower);
    }

    public static void setIntakePower(double newPower) {
        new InstantCommand(() -> intakePower = newPower);
    }
}
