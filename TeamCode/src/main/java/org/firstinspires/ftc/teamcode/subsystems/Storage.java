package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

public class Storage implements Subsystem {
    public static final Storage INSTANCE = new Storage();
    private static boolean manualMode = true;
    private static boolean positionMode = false;
    private static double manualPower = 0;
    private final static MotorEx spin = new MotorEx("motorExp0").brakeMode();
    private static DigitalChannel limitSwitch;
    private static NormalizedColorSensor colorSensor;
    private static double currentPosition;
    private static double targetPosition;
    private static final double DELTA_TICKS = 185;
    private static final double OUTTAKE_POSITION = DELTA_TICKS + DELTA_TICKS / 2;
    private static boolean lastState = false;

    public static ControlSystem controller = ControlSystem.builder()
            .posPid(0.0075, 0, 0)
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
        currentPosition = spin.getCurrentPosition();
        targetPosition = currentPosition;

        limitSwitch = ActiveOpMode.hardwareMap().get(DigitalChannel.class,
                "limitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        colorSensor = ActiveOpMode.hardwareMap().get(NormalizedColorSensor.class,
                "colorSensor");
    }

    @Override
    public void periodic() {
        if (manualMode) {
            spin.setPower(manualPower);
        } else if (positionMode) {
            double testPower = controller.calculate(new KineticState(targetPosition));
            if (Math.abs(testPower) > 0.05) {
                spin.setPower(testPower);
            } else {
                spin.setPower(0);
            }
        }
//        if (wasJustPressed()) {
//            resetEncoderAtOuttake();
//        }
    }

    public static Command spinToNextIntakeIndex() {
        return new LambdaCommand()
                .setStart(() -> {
                    manualMode = false;
                    positionMode = true;
                    double startPos = currentPosition + 10;

                    double remainder = startPos % DELTA_TICKS;
                    if (remainder < 0) remainder += DELTA_TICKS;

                    double ticksToMove = DELTA_TICKS - remainder;

                    targetPosition = startPos + ticksToMove;
                })
                .setIsDone(() -> true)
                .setStop(interrupted -> {
                })
                .requires(Storage.INSTANCE)
                .setInterruptible(true)
                .named("Spin to next index");
    }

    public static Command spinToNextOuttakeIndex() {
        return new LambdaCommand()
                .setStart(() -> {
                    manualMode = false;
                    positionMode = true;
                    double startPos = currentPosition - 10;

                    double remainder = startPos % DELTA_TICKS;
                    if (remainder < 0) remainder += DELTA_TICKS;

                    double ticksToMove = (DELTA_TICKS / 2.0) - remainder;
                    if (ticksToMove >= 0) ticksToMove -= DELTA_TICKS;

                    targetPosition = startPos + ticksToMove;
                })
                .setIsDone(() -> true)
                .setStop(interrupted -> {
                })
                .requires(Storage.INSTANCE)
                .setInterruptible(true)
                .named("Spin to next index");
    }

    public static Command setManualPowerCommand(double newPower) {
        return new InstantCommand(() -> setManualPower(newPower));
    }

    public static Command setManualModeCommand(boolean newMode) {
        return new InstantCommand(() -> setManualMode(newMode));
    }

    public static Command setPositionModeCommand(boolean newMode) {
        return new InstantCommand(() -> setPositionMode(newMode));
    }

    public static Command resetEncoderCommand() {
        return new InstantCommand(() -> resetEncoder(0));
    }

    public static Command resetEncoderAtOuttakeCommand() {
        return new InstantCommand(() -> resetEncoder(OUTTAKE_POSITION));
    }

    private static void setManualPower(double newPower) {
        manualPower = newPower;
    }

    private static void setPositionMode(boolean newBoolean) {
        positionMode = newBoolean;
    }

    private static void setManualMode(boolean newMode) {
        manualMode = newMode;
    }

    private static void resetEncoder(double newPosition) {
        spin.setCurrentPosition(newPosition);
        targetPosition = newPosition;
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

    public static boolean wasJustPressed() {
        boolean currentState = limitSwitch.getState();
        boolean justPressed = currentState && !lastState;
        lastState = currentState;
        return justPressed;
    }
}
