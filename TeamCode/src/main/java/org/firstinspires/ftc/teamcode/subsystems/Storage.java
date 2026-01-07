package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.utils.Logger;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

public class Storage implements Subsystem {
    public static final Storage INSTANCE = new Storage();
    private static boolean manualMode = true;
    private static boolean pidControlMode = false;
    private static double manualPower = 0;
    private final static MotorEx spin = new MotorEx("motorExp0").brakeMode();
    private static DigitalChannel limitSwitch;
    private static NormalizedColorSensor colorSensor;
    private static int index = 0;
    private static double startPos = 0;
    private static final double TICKS = 188.2166666667;

    static ControlSystem controller = ControlSystem.builder()
            .posPid(0.01, 0, 0)
            .basicFF(0)
            .build();

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
        // limitSwitch = ActiveOpMode.hardwareMap().get(DigitalChannel.class,
        // "limitSwitch");
        // limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        //
        // colorSensor = ActiveOpMode.hardwareMap().get(NormalizedColorSensor.class,
        // "sensor_color");
        //
        // ((SwitchableLight) colorSensor).enableLight(false);
    }

    @Override
    public void periodic() {

        // Track Indexes
        // if (wasJustPressed()) {
        // index++;
        // if (index >= STATES.length) {
        // index = 0;
        // }
        // }

        // Should be the only location we're setting the power, default to manualMode
        if (manualMode) {
            spin.setPower(manualPower);
        } else if (pidControlMode) {
            // Update the control system target
            spin.setPower(controller.calculate(spin.getState()));
        }

        // Write Telemetry
        Logger.add("Storage", Logger.Level.DEBUG, "ticks: " + spin.getCurrentPosition());
        Logger.add("Storage", Logger.Level.DEBUG, "pid?" + pidControlMode +  "power: " + controller.calculate(spin.getState()));
        Logger.add("Storage", Logger.Level.DEBUG, "manual?" + manualMode +  "power: " + manualPower);

        // ActiveOpMode.telemetry().addLine("Limit: " + limitSwitch.getState());
        // ActiveOpMode.telemetry().addLine("Color: " + getColor().red + ", " +
        // getColor().green + ", " + getColor().blue );
    }

    public static Command spinToNextIntakeIndex() {
        return new LambdaCommand()
                .setStart(() -> {
                    manualMode = false;
                    pidControlMode = true;
                    startPos = spin.getCurrentPosition() + 10;
                    double nextPos = startPos + (TICKS - (startPos % TICKS));
                    controller.setGoal(new KineticState(nextPos));
                })
                .setIsDone(() -> true)
                .setStop(interrupted -> {})
                .requires(Storage.INSTANCE)
                .setInterruptible(true)
                .named("Spin to next index");
    }

    public static Command spinToNextOuttakeIndex() {
        return new LambdaCommand()
                .setStart(() -> {
                    manualMode = false;
                    pidControlMode = true;
                    startPos = spin.getCurrentPosition() + 10;
                    double ticksToMove = TICKS/2.0 - startPos % TICKS;

                    if (ticksToMove <= 0) {
                        ticksToMove += TICKS;
                    }
                    double nextPos = startPos + ticksToMove;
                    controller.setGoal(new KineticState(nextPos));})
                .setIsDone(() -> true)
                .setStop(interrupted -> {
                })
                .requires(Storage.INSTANCE)
                .setInterruptible(true)
                .named("Spin to next index");
    }

    public static void setManualPower(double newPower) {
        manualPower = newPower;
    }

    public static void setPidControlMode(boolean newBoolean) {
        pidControlMode = newBoolean;
    }

    public static void setManualMode(boolean newMode) {
        manualMode = newMode;
    }

    public static void resetEncoder() {
        spin.zero();
    }

//    public static NormalizedRGBA getColor() {
//        return colorSensor.getNormalizedColors();
//    }
//
//    public static State readColor() {
//        NormalizedRGBA colors = getColor();
//
//        double red = colors.red;
//        double green = colors.green;
//        double blue = colors.blue;
//
//        double sum = red + green + blue;
//
//        double r = red / sum;
//        double g = green / sum;
//        double b = blue / sum;
//
//        if (r > 0.35 && b > 0.35 && g < 0.25) {
//            return State.PURPLE;
//        }
//        if (g > 0.45 && r < 0.3 && b < 0.3) {
//            return State.GREEN;
//        }
//        return State.NONE;
//    }
//
//    public boolean wasJustPressed() {
//        boolean current = limitSwitch.getState();
//        boolean triggered = current && !lastState;
//        lastState = current;
//        return triggered;
//    }
//
//    public static Command prioritySpin(int targetIndex) {
//        return new LambdaCommand()
//                .setStart(() -> {
//                    manualMode = false;
//                    runPower = .6;
//                })
//                .setIsDone(() -> index == targetIndex)
//                .setStop(interrupted -> {
//                    runPower = 0;
//                    manualPower = 0;
//                    manualMode = true;
//                })
//                .requires(Storage.INSTANCE)
//                .setInterruptible(false)
//                .named("Priority Spin → " + targetIndex);
//    }
//
//    public static Command lazySpin(int targetIndex) {
//        return new LambdaCommand()
//                .setStart(() -> {
//                    manualMode = false;
//                    runPower = .6;
//                })
//                .setIsDone(() -> index == targetIndex)
//                .setStop(interrupted -> {
//                    if (!interrupted) {
//                        runPower = 0;
//                        manualPower = 0;
//                        manualMode = true;
//                    }
//                })
//                .requires(Storage.INSTANCE)
//                .setInterruptible(true)
//                .named("Lazy Spin → " + targetIndex);
//    }
//
//    public static Command spinToState(State targetState) {
//        int targetIndex = -1;
//        for (int i = 0; i < STATES.length; i++) {
//            if (STATES[i] == targetState) {
//                targetIndex = i;
//                break;
//            }
//        }
//        if (targetIndex == -1) {
//            return new NullCommand();
//        }
//        return prioritySpin(targetIndex);
//    }
//
//    public static Command reIndex() {
//
//        Command[] steps = new Command[STATES.length * 2];
//        int cmdIndex = 0;
//
//        for (int i = 0; i < STATES.length; i++) {
//            final int target = i;
//
//            steps[cmdIndex++] = prioritySpin(target);
//
//            steps[cmdIndex++] = new LambdaCommand()
//                    .setStart(() -> STATES[target] = readColor())
//                    .setIsDone(() -> true)
//                    .named("read color at " + target);
//        }
//
//        return new SequentialGroup(steps)
//                .named("reIndex");
//    }
//
//    public static Command clearCurrentIndex() {
//        return new LambdaCommand()
//                .setStart(() -> STATES[index] = State.NONE)
//                .setIsDone(() -> true)
//                .named("clear current index");
//    }
//
//    public static Command clearIndexState(int targetIndex) {
//        return new LambdaCommand()
//                .setStart(() -> STATES[targetIndex] = State.NONE)
//                .setIsDone(() -> true)
//                .named("clear state of " + targetIndex);
//    }
//
//    public static Command resetIndexStates() {
//
//        Command[] clears = new Command[STATES.length];
//
//        for (int i = 0; i < STATES.length; i++) {
//            clears[i] = clearIndexState(i);
//        }
//        return new SequentialGroup(clears)
//                .named("reset index states");
//    }


//    public static Command spinForwardTicks(int ticks) {
//        return new LambdaCommand()
//                .setStart(() -> {
//                    manualMode = false;
//                    pidControlMode = true;
//                    startPos = spin.getCurrentPosition();
//                    controller.setGoal(new KineticState(startPos + ticks));
//                })
//                .setUpdate(() -> {})
//                .setIsDone(() -> manualMode)
//                .setStop(interrupted -> {})
//                .requires(Storage.INSTANCE)
//                .setInterruptible(false)
//                .named("Spin Forward " + ticks + " Ticks");
//    }
}
