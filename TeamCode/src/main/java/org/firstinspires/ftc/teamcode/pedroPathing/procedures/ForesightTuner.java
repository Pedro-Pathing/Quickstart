package org.firstinspires.ftc.teamcode.pedroPathing.procedures;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.function.Function;

public class ForesightTuner extends Procedure {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;

    public ForesightTuner(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction) {
        super("Foresight Tuner", "A procedure for tuning the Foresight Algorithm.");
        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
    }

    @Override
    public void run() throws InterruptedException {
        Inputs distanceInput = inputs("Distance", "The distance to drive in inches for the Max Achievable Forward and Strafe Identifiers");
        Inputs.Field<Double> distance = distanceInput.d("Distance").withDefault(48.0);
        awaitInputs(distanceInput);

        double forwardVelocity = runOpMode(new ForwardVelocity(localizerFunction, drivetrainFunction, distance.get()));
        double strafeVelocity = runOpMode(new StrafeVelocity(localizerFunction, drivetrainFunction, distance.get()));

        Inputs velocityInput = inputs("Velocity", "The velocity to drive to in inches for the Max Achievable Forward and Strafe Deceleration Identifiers");
        Inputs.Field<Double> velocity = velocityInput.d("Distance").withDefault(48.0);
        awaitInputs(velocityInput);

        double forwardDeceleration = runOpMode(new ForwardDeceleration(localizerFunction, drivetrainFunction, velocity.get()));
        double strafeDeceleration = runOpMode(new StrafeDeceleration(localizerFunction, drivetrainFunction, velocity.get()));

        result("maxAchievableForwardVelocity", forwardVelocity);
        result("maxAchievableStrafeVelocity", strafeVelocity);
        result("naturalForwardDeceleration", forwardDeceleration);
        result("naturalStrafeDeceleration", strafeDeceleration);
    }
}

class ForwardVelocity extends TuningOpMode<Double> {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;
    double distance;
    private final ArrayDeque<Double> velocities = new ArrayDeque<>();
    public static double RECORD_NUMBER = 10;

    public ForwardVelocity(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction, double distance) {
        super("Max Forward Velocity", "A tuner for finding the maximum achievable forward velocity.", false);
        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
        this.distance = distance;
    }

    @Override
    protected Double runTuningOpMode() {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

        boolean end = false;

        localizer.setPose(Pose.zero());
        localizer.update();

        DrivePowers power = new DrivePowers(1,0,0);

        for (int i = 0; i < RECORD_NUMBER; i++) {
            velocities.add(0.0);
        }

        waitForStart();

        while (!end) {
            localizer.update();
            if (Math.abs(localizer.pose().x()) > distance) {
                end = true;
                drivetrain.drive(DrivePowers.zero(), true);
            } else {
                drivetrain.drive(power, true);
                double currentVelocity = Math.abs(localizer.twist().toVector2D().x());
                velocities.addLast(currentVelocity);
                velocities.removeFirst();
            }
        }

        drivetrain.drive(DrivePowers.zero(), true);
        double average = 0;
        for (double velocity : velocities) {
                average += velocity;
        }
        average /= velocities.size();
        return average;
    }
}

class StrafeVelocity extends TuningOpMode<Double> {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;
    double distance;
    private final ArrayDeque<Double> velocities = new ArrayDeque<>();
    public static double RECORD_NUMBER = 10;

    public StrafeVelocity(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction, double distance) {
        super("Max Strafe Velocity", "A tuner for finding the maximum achievable strafe velocity.", false);
        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
        this.distance = distance;
    }

    @Override
    protected Double runTuningOpMode() {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

        boolean end = false;

        localizer.setPose(Pose.zero());
        localizer.update();

        DrivePowers power = new DrivePowers(0,1,0);

        for (int i = 0; i < RECORD_NUMBER; i++) {
            velocities.add(0.0);
        }

        waitForStart();

        while (!end) {
            localizer.update();
            if (Math.abs(localizer.pose().y()) > distance) {
                end = true;
                drivetrain.drive(DrivePowers.zero(), true);
            } else {
                drivetrain.drive(power, false);
                double currentVelocity = Math.abs(localizer.twist().toVector2D().y());
                velocities.addLast(currentVelocity);
                velocities.removeFirst();
            }
        }

        drivetrain.drive(DrivePowers.zero(), true);
        double average = 0;
        for (double velocity : velocities) {
            average += velocity;
        }
        average /= velocities.size();
        return average;
    }
}

class ForwardDeceleration extends TuningOpMode<Double> {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;
    double velocity;
    private final ArrayList<Double> accelerations = new ArrayList<>();

    private double previousVelocity;
    private long previousTimeNano;

    private boolean stopping;

    public ForwardDeceleration(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction, double velocity) {
        super("Forward Deceleration", "A tuner for finding the deceleration of the robot when moving forward.", false);
        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
        this.velocity = velocity;
    }

    @Override
    protected Double runTuningOpMode() {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

        boolean end = false;

        localizer.setPose(Pose.zero());
        localizer.update();

        DrivePowers power = new DrivePowers(1, 0, 0);
        drivetrain.drive(power, false);

        waitForStart();

        while (!end) {
            if (!stopping) {
                if (localizer.twist().toVector2D().x() > velocity) {
                    previousVelocity = localizer.twist().toVector2D().x();
                    previousTimeNano = System.nanoTime();
                    stopping = true;
                    drivetrain.drive(DrivePowers.zero(), true);
                }
            } else {
                double currentVelocity = localizer.twist().toVector2D().x();
                accelerations.add((currentVelocity - previousVelocity) / ((System.nanoTime() - previousTimeNano) / Math.pow(10.0, 9)));
                previousVelocity = currentVelocity;
                previousTimeNano = System.nanoTime();
                if (currentVelocity < 0.1) {
                    end = true;
                }
            }
        }

        drivetrain.drive(DrivePowers.zero(), true);
        double average = 0;
        for (double acceleration : accelerations) {
            average += acceleration;
        }
        average /= accelerations.size();
        return Math.abs(average);
    }
}

class StrafeDeceleration extends TuningOpMode<Double> {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;
    double velocity;
    private final ArrayList<Double> accelerations = new ArrayList<>();

    private double previousVelocity;
    private long previousTimeNano;

    private boolean stopping;

    public StrafeDeceleration(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction, double velocity) {
        super("Strafe Deceleration", "A tuner for finding the deceleration of the robot when moving sideways.", false);
        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
        this.velocity = velocity;
    }

    @Override
    protected Double runTuningOpMode() {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

        boolean end = false;

        localizer.setPose(Pose.zero());
        localizer.update();

        DrivePowers power = new DrivePowers(0, 1, 0);
        drivetrain.drive(power, false);

        waitForStart();

        while (!end) {
            if (!stopping) {
                if (localizer.twist().toVector2D().y() > velocity) {
                    previousVelocity = localizer.twist().toVector2D().y();
                    previousTimeNano = System.nanoTime();
                    stopping = true;
                    drivetrain.drive(DrivePowers.zero(), true);
                }
            } else {
                double currentVelocity = localizer.twist().toVector2D().y();
                accelerations.add((currentVelocity - previousVelocity) / ((System.nanoTime() - previousTimeNano) / Math.pow(10.0, 9)));
                previousVelocity = currentVelocity;
                previousTimeNano = System.nanoTime();
                if (currentVelocity < 0.1) {
                    end = true;
                }
            }
        }

        drivetrain.drive(DrivePowers.zero(), true);
        double average = 0;
        for (double acceleration : accelerations) {
            average += acceleration;
        }
        average /= accelerations.size();
        return Math.abs(average);
    }
}
