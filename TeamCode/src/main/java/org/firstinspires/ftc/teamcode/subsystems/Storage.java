package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.utils.Logger;

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
    private static boolean pidControlMode = false;
    private static double manualPower = 0;
    private final static MotorEx spin = new MotorEx("motorExp0").brakeMode();
    private static DigitalChannel limitSwitch;
    private static NormalizedColorSensor colorSensor;
    private static double startPos = 0;
    private static final double TICKS = 185;

    private static boolean lastState = false;

    public static boolean getManualMode() {
        return manualMode;
    }

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
        spin.zero();
        controller.setGoal(new KineticState(0));


        limitSwitch = ActiveOpMode.hardwareMap().get(DigitalChannel.class,
                "limitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        colorSensor = ActiveOpMode.hardwareMap().get(NormalizedColorSensor.class,
                "colorSensor");

    }

    @Override
    public void periodic() {

//        if (wasJustPressed()) {
//            resetEncoderAtOuttake();
//        }

        if (manualMode) {
            spin.setPower(manualPower);
        } else if (pidControlMode) {
//            double testPower = controller.calculate(spin.getState());
//            if (Math.abs(testPower) > 0.05) {
//                spin.setPower(testPower);
//            } else {
//                spin.setPower(0);
//            }
        }

        // Write Telemetry
        Logger.add("Storage", Logger.Level.DEBUG, "ticks: " + spin.getCurrentPosition());
        Logger.add("Storage", Logger.Level.DEBUG, "pid?" + pidControlMode + "power: " + controller.calculate(spin.getState()));
        Logger.add("Storage", Logger.Level.DEBUG, "manual?" + manualMode + "power: " + manualPower);
        Logger.add("Storage", Logger.Level.DEBUG, "limit switch" + limitSwitch.getState());
        Logger.add("Storage", Logger.Level.DEBUG, "Color: " + getColor().red + ", " + getColor().green + ", " + getColor().blue);
    }

    public static Command setManualPowerCommand(double newPower) {
        return new InstantCommand(() -> setManualPower(newPower));
    }

    public static Command setManualModeCommand(boolean newMode) {
        return new InstantCommand(() -> setManualMode(newMode));
    }

    public static Command setPIDMode(boolean newMode) {
        return new InstantCommand(() -> setPidControlMode(newMode));
    }

    public static Command resetEncoderCommand() {
        return new InstantCommand(Storage::resetEncoder);
    }

    public static Command spinToNextIntakeIndex() {
        return new LambdaCommand()
                .setStart(() -> {
                    manualMode = false;
                    pidControlMode = true;
                    startPos = spin.getCurrentPosition() + 10;

                    double remainder = startPos % TICKS;
                    if (remainder < 0) remainder += TICKS;

                    double ticksToMove = TICKS - remainder;

                    double nextPos = startPos + ticksToMove;
                    controller.setGoal(new KineticState(nextPos));
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
                    manualMode = true;
                    manualPower = 0.1;
                    manualMode = false;
                    pidControlMode = true;
                    startPos = spin.getCurrentPosition() - 10;

                    double remainder = startPos % TICKS;
                    if (remainder < 0) remainder += TICKS;

                    double ticksToMove = (TICKS / 2.0) - remainder;

                    if (ticksToMove >= 0) {
                        ticksToMove -= TICKS;
                    }
                    double nextPos = startPos + ticksToMove;
                    controller.setGoal(new KineticState(nextPos));
                })
                .setIsDone(() -> true)
                .setStop(interrupted -> {
                })
                .requires(Storage.INSTANCE)
                .setInterruptible(true)
                .named("Spin to next index");
    }
    private static void setManualPower(double newPower) {
        manualPower = newPower;
    }

    private static void setPidControlMode(boolean newBoolean) {
        pidControlMode = newBoolean;
    }

    private static void setManualMode(boolean newMode) {
        manualMode = newMode;
    }

    private static void resetEncoder() {
        spin.zero();
        controller.setGoal(new KineticState(0));
    }

    private static void resetEncoderAtOuttake() {
        spin.setCurrentPosition(270);
        controller.setGoal(new KineticState(270));
    }

    public static Command resetEncoderAtOuttakeCommand() {
        return new InstantCommand(Storage::resetEncoderAtOuttake);
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
