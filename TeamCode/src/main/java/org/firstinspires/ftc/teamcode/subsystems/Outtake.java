package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Logger;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

public class Outtake implements Subsystem {

    public static final Outtake INSTANCE = new Outtake();
    private static final MotorEx outtake = new MotorEx("motorExp3").reversed().floatMode();
    private Servo outtakeServo;
    private static boolean runDown = false;

    private static final ControlSystem controller = ControlSystem.builder()
            .velPid(0.035, 0, 0)
            .basicFF(0.00030, 0, 0.075)
            .build();
    public static Command off = new RunToVelocity(controller, 0.0).requires(INSTANCE).named("FlywheelOff");
    public static Command on = new RunToVelocity(controller, 2050).requires(INSTANCE).named("FlywheelOn");

    private static void setRunDown(boolean newBoolean) {
        runDown = newBoolean;
    }

    public static Command setRunDownCommand(boolean newBoolean) {
        return new InstantCommand(() -> setRunDown(newBoolean));
    }

    private static double outtakePower = 0;

    private static void setOuttakePower(double newPower) {
        outtake.setPower(newPower);
    }

    public static Command setOuttakePowerCommand(double newPower) {
        return new InstantCommand(() -> setOuttakePower(newPower));
    }

    @Override
    public void initialize() {
        outtakeServo = ActiveOpMode.hardwareMap().servo.get("servo5");
        setRunDownCommand(true).schedule();
    }
    @Override
    public void periodic() {
        outtakeServo.setPosition(0);
        //outtake.setPower(outtakePower);
        double testPower = controller.calculate(outtake.getState());
        Logger.add("Outtake", Logger.Level.DEBUG, "velocity: " + outtake.getVelocity() + "power: " + testPower );
        if (runDown) {
            outtake.setPower(0);
        } else {
            if (Math.abs(testPower) > 0.05){
                outtake.setPower(testPower);
            } else {
                outtake.setPower(0);
            }
        }
    }
}
