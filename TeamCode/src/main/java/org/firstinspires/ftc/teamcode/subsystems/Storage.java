package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

public class Storage implements Subsystem {
    public static final Storage INSTANCE = new Storage();
    private static boolean manualMode = true;
    private static double manualPower = 1;
    private static MotorEx spin = new MotorEx("motor2")
            .brakeMode();

    private DigitalChannel limitSwitch;
    private static boolean lastState = false;
    private static int index = 0;


    private enum PositionState {
        PURPLE,
        GREEN,
        NONE
    }

    @Override
    public void initialize() {
        limitSwitch = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "limitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void periodic() {
        if (manualMode) {
            spin.setPower(manualPower);
        } else {
        }

        // Handle state logic
        if (wasJustPressed()) {
            if (index == 3){
                index = 0;
            } else {
                index++;
            }
        }
        // Write Telemetry
        ActiveOpMode.telemetry().addData("Storage Position", spin.getCurrentPosition());
        ActiveOpMode.telemetry().addData("manualmode?", manualMode);
        ActiveOpMode.telemetry().addData("manualpower", manualPower);
        ActiveOpMode.telemetry().addData("wasJustPressed?", wasJustPressed());
        ActiveOpMode.telemetry().addData("pressed?", limitSwitch.getState());

        ActiveOpMode.telemetry().update();
    }
    public static void setManualPower(double newPower) {
        manualPower = newPower;
    }
    public static void setManualMode(boolean newMode) {
        manualMode = newMode;
    }
    public static void resetEncoder(){
        spin.zero();
    }
    public boolean wasJustPressed() {
        boolean current = limitSwitch.getState();
        boolean triggered = current && !lastState;
        lastState = current;
        return triggered;
    }
}
