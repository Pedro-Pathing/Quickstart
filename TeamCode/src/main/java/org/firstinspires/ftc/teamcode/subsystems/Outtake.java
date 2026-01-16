package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
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
    private static final MotorEx outtake = new MotorEx("motorExp2").reversed().floatMode();
    private Servo outtakeServo;
    private static double outtakePower = 0;

    private PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;


    private static boolean runDown = false;

    private static final ControlSystem controller = ControlSystem.builder()
            .velPid(0.045, 0, 0)
            .basicFF(0.0005, 0, 0.1)
            .build();
    public static Command off = new RunToVelocity(controller, 0.0).requires(INSTANCE).named("FlywheelOff");
    public static Command on = new RunToVelocity(controller, 2300).requires(INSTANCE).named("FlywheelOn");

    private static void setRunDown(boolean newBoolean) {
        runDown = newBoolean;
    }

    public static boolean isOuttakeAbove(double targetVelo){
        return outtake.getVelocity() > targetVelo;
    }

    public static Command setRunDownCommand(boolean newBoolean) {
        return new InstantCommand(() -> setRunDown(newBoolean));
    }


    @Override
    public void initialize() {
        outtakeServo = ActiveOpMode.hardwareMap().servo.get("servo5");
        setRunDownCommand(true).schedule();
    }
    @Override
    public void periodic() {
        updateSignals();
        outtakeServo.setPosition(0);
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

    private void updateSignals(){
        panelsTelemetry.getTelemetry().addData("velo", outtake.getVelocity());
        panelsTelemetry.getTelemetry().addData("power", outtake.getPower());
        panelsTelemetry.getTelemetry().update();
    }

    private static void setOuttakePower(double newPower) {
        outtakePower = newPower;
    }

    public static Command setOuttakePowerCommand(double newPower) {
        return new InstantCommand(() -> setOuttakePower(newPower));
    }
}
