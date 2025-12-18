package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

public class Storage implements Subsystem {
    public static final Storage INSTANCE = new Storage();
    private static boolean manualMode = true;
    private static double manualPower = 1;
    private static double runPower = 0;
    private static MotorEx spin = new MotorEx("motor2").brakeMode();

    private static DigitalChannel limitSwitch;
    private static NormalizedColorSensor colorSensor;
    private static boolean lastState = false;
    private static int index = 0;
    public static final State[] STATES = {
            State.NONE,
            State.NONE,
            State.NONE
    };

    public enum State {
        PURPLE,
        GREEN,
        NONE,
        BALL,
    }

    @Override
    public void initialize() {
        limitSwitch = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "limitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        colorSensor = ActiveOpMode.hardwareMap().get(NormalizedColorSensor.class, "sensor_color");

        ((SwitchableLight) colorSensor).enableLight(false);
        // TODO: optimize this so that the light only turns on when we need it to
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

        // Should be the only location we're setting the power, default to manualMode
        if (manualMode) {
            spin.setPower(manualPower);
        } else {
            spin.setPower(runPower);
        }

        // Write Telemetry
        ActiveOpMode.telemetry().addLine("Storage | Ticks: " + spin.getCurrentPosition() + " | Index: " + index);
        ActiveOpMode.telemetry().addLine("Manual: " + manualMode + " | Power: " + manualPower);
        ActiveOpMode.telemetry().addLine("Limit: " + limitSwitch.getState());
        ActiveOpMode.telemetry().addLine("Color: " + getColor().red + ", " + getColor().green + ", " + getColor().blue );

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

    public static NormalizedRGBA getColor() {
        return colorSensor.getNormalizedColors();
    }
    public static State readColor() {
        NormalizedRGBA colors = getColor();

        double red = colors.red;
        double green = colors.green;
        double blue = colors.blue;

        double sum = red + green + blue;

        double r = red / sum;
        double g = green / sum;
        double b = blue / sum;

        if (r > 0.35 && b > 0.35 && g < 0.25) {
            return State.PURPLE;
        }
        if (g > 0.45 && r < 0.3 && b < 0.3) {
            return State.GREEN;
        }
        return State.NONE;
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
                    runPower = 0;
                    manualPower = 0;
                    manualMode = true;
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
                    if (!interrupted) {
                        runPower = 0;
                        manualPower = 0;
                        manualMode = true;
                    }
                })
                .requires(Storage.INSTANCE)
                .setInterruptible(true)
                .named("Lazy Spin → " + targetIndex);
    }

    public static Command spinToState(State targetState) {
        int targetIndex = -1;
        for (int i = 0; i < STATES.length; i++) {
            if (STATES[i] == targetState) {
                targetIndex = i;
                break;
            }
        }
        if (targetIndex == -1) {
            return new NullCommand();
        }
        return prioritySpin(targetIndex);
    }
    public static Command reIndex() {

        Command[] steps = new Command[STATES.length * 2];
        int cmdIndex = 0;

        for (int i = 0; i < STATES.length; i++) {
            final int target = i;

            steps[cmdIndex++] = prioritySpin(target);

            steps[cmdIndex++] = new LambdaCommand()
                    .setStart(() -> STATES[target] = readColor())
                    .setIsDone(() -> true)
                    .named("read color at " + target);
        }

        return new SequentialGroup(steps)
                .named("reIndex");
    }

    public static Command clearCurrentIndex() {
        return new LambdaCommand()
                .setStart(() -> STATES[index] = State.NONE)
                .setIsDone(() -> true)
                .named("clear current index");
    }
    public static Command clearIndexState(int targetIndex) {
        return new LambdaCommand()
                .setStart(() -> STATES[targetIndex] = State.NONE)
                .setIsDone(() -> true)
                .named("clear state of " + targetIndex);
    }
    public static Command resetIndexStates() {

        Command[] clears = new Command[STATES.length];

        for (int i = 0; i < STATES.length; i++) {
            clears[i] = clearIndexState(i);
        }
        return new SequentialGroup(clears)
                .named("reset index states");
    }
}
