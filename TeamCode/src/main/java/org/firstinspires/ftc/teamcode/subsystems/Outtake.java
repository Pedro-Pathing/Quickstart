package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.control.KineticState;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import org.firstinspires.ftc.teamcode.utils.Logger;

public class Outtake implements Subsystem {

    public static final Outtake INSTANCE = new Outtake();
    private static final MotorEx outtake = new MotorEx("motorExp2").reversed().floatMode();
    private static Servo hoodServo;
    private static Servo traverseServo;
    private static final PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    private static double targetVelocity = 2300;
    private static double currentVelocity = 0;
    private static double manualPower = 0;
    private static boolean velocityMode = false;
    private static boolean manualMode = false;
    /*
    TODO: retune these values
     ks -> minimum for movement; kV -> choose a velocity, minimum to reach that movement;
     kP until reaches other velocites
     */
    private static final ControlSystem controller = ControlSystem.builder()
            .velPid(0.045, 0, 0)
            .basicFF(0.0005, 0, 0.1)
            .build();

    public static Command off = new InstantCommand(() -> {
        velocityMode = false;
        manualMode = false;
    });
    public static Command on = new InstantCommand(() -> {
        manualMode = false;
        velocityMode = true;
    });

    public static Command setOuttakeManualPowerCommand(double newPower) {
        return new InstantCommand(() -> setManualPower(newPower));
    }

    public static Command setOuttakeManualModeCommand(boolean newMode) {
        return new InstantCommand(() -> setManualMode(newMode));
    }

    @Override
    public void initialize() {
        manualMode = false;
        velocityMode = false;
        manualPower = 0;
    }

    @Override
    public void periodic() {
        currentVelocity = outtake.getVelocity();

        if (manualMode) {
            outtake.setPower(manualPower);
        } else if (velocityMode){
            controller.setGoal(new KineticState(0, targetVelocity));
            double velocityPower = controller.calculate(new KineticState(0, currentVelocity));

            outtake.setPower(velocityPower);
        } else {
            outtake.setPower(0);
        }

        Logger.panelsLog("velo", currentVelocity);
        Logger.panelsLog("power", outtake.getPower());
    }

    public static boolean reachedTargetVelocity(){
        return outtake.getVelocity() > targetVelocity;
    }

    public static void setTargetVelocity(double newTargetVelocity){
        Outtake.targetVelocity = newTargetVelocity;
    }

    public static void setManualMode(boolean newMode){
        manualMode = newMode;
    }

    public static void setManualPower(double newPower){
        manualPower = newPower;
    }
}
