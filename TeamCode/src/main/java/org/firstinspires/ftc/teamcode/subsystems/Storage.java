package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

public class Storage implements Subsystem {
    public static final Storage INSTANCE = new Storage();
    private static boolean manualMode = true;
    private static double manualPower = 1;
    private static double runPower = 0;
    private static MotorEx spin = new MotorEx("motor2").brakeMode();

    private DigitalChannel limitSwitch;
    private static boolean lastState = false;
    private static int index = 0;
    private static final State[] STATES = {
            State.NONE,
            State.NONE,
            State.NONE
    };

    // PURPLE & GREEN should take priority
    // If the state is unknown, it should be BALL
    // At which time we should queue for a read to determine if it's PURPLE, GREEN, or NONE
    private enum State {
        PURPLE,
        GREEN,
        NONE,
        BALL,
    }

    @Override
    public void initialize() {
        limitSwitch = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "limitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void periodic() {

        // Track Indexes
        if (wasJustPressed()) {
            index++;
            if (index >= STATES.length) {
                index = 0;
            }
        }

        if (manualMode) {
            spin.setPower(manualPower);
        } else {
            spin.setPower(runPower);
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

    public static Command prioritySpin(int targetIndex) {
        return new LambdaCommand()
                .setStart(() -> {
                    manualMode = false;
                    runPower = .6;
                })
                .setIsDone(() -> index == targetIndex)

                .setStop(interrupted -> {
                    manualPower = 0;
                    manualMode = true;
                    runPower = 0;
                })
                .requires(Storage.INSTANCE)
                .setInterruptible(false)
                .named("Priority Spin → " + targetIndex);
    }
    public static Command lazySpin(int targetIndex) {
        return new LambdaCommand()
                .setStart(() -> {
                    manualMode = false;
                    runPower = .6;
                })
                .setIsDone(() -> index == targetIndex)

                .setStop(interrupted -> {
                    manualPower = 0;
                    manualMode = true;
                    runPower = 0;
                })
                .requires(Storage.INSTANCE)
                .setInterruptible(true)
                .named("Lazy Spin → " + targetIndex);
    }
}
