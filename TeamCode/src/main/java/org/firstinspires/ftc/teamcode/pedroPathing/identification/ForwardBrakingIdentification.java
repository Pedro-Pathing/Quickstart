package org.firstinspires.ftc.teamcode.pedroPathing.identification;

import static com.pedropathing.utils.Angle.normalizeSigned;
import static com.pedropathing.utils.Utils.quadraticFit;

import android.annotation.SuppressLint;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.utils.Angle;
import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

/**
 * This is the Forward Braking Identification. It runs the robot forward and backward at various
 * power levels, recording the robot’s velocity and position immediately before braking.
 * The motors are then set to a very small reverse power, which actives a harsh velocity-proportional force.
 * Once the robot comes to a complete stop, the tuner measures the stopping distance.
 * Using the collected data, it generates a velocity-vs-stopping-distance graph and fits a quadratic curve to model the braking behavior.
 *
 * @author Jacob Ophoven - 12649 Code Blooded
 * @version 8/11/2026
 */
@TeleOp(group = "2")
public class ForwardBrakingIdentification extends OpMode {
    private static double[] POWERS;
    public static double MAX_BRAKE_TIME = 5; //seconds, the robot shouldn't take longer than this to brake

    public static int trials = 12;
    public static double maxPower = 0.7;
    public static double minPower = 0.2;
    public static double bias = 1.5; // how much it favors doing trials with higher powers
    public static double brakingPower = 0.001;
    public static int TILES_IN_FRONT_OF_ROBOT = 3; // Must be at least 3
    public static double IDLE_SECONDS = 1; //if your robot tips, increase this to add more delay between trials

    private final ElapsedTime timer = new ElapsedTime();

    private final List<double[]> velocityToBrakingDistance = new ArrayList<>();
    private State state = State.DRIVE;
    private int iteration = 0;
    private int direction;
    private double power;

    private Vector2D startPosition;
    private double measuredVelocity;

    private Follower follower;
    private VoltageSensor voltageSensor;

    @Override
    public void init() {
        POWERS = biasedGradient(trials, maxPower, minPower, bias);

        follower = Constants.create(hardwareMap);
        follower.setPose(Pose.zero());
        follower.update();
        voltageSensor = hardwareMap.getAll(VoltageSensor.class).iterator().next();

        recordBrakeData();
    }

    @Override
    public void start() {
        follower.setPose(Pose.zero());
        follower.update();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("The robot will need " + TILES_IN_FRONT_OF_ROBOT + " tiles in front of it to run.");
        telemetry.addLine("It will drive at different powers forwards and backwards, measuring braking distance while correcting its heading.");
        telemetry.addLine("Make sure you have enough room.");
        telemetry.addLine("After stopping, the forward linear and quadratic braking coefficients will be displayed.");
        telemetry.update();
        follower.update();
    }

    private double getHeadingPower() {
        double angularVel = follower.velocity().omega;
        double brakeDist = Constants.foresightConfig.headingBrakeCoefficients.get().x() * angularVel +
                Constants.foresightConfig.headingBrakeCoefficients.get().y() * angularVel * angularVel * Math.signum(angularVel);
        double headingError = Angle.normalizeSigned(-follower.pose().heading());
        double error = headingError - brakeDist;
        return Utils.clamp(Constants.foresightConfig.headingFeedback.get().plus(Constants.foresightConfig.headingStaticFF.get())
                .calculate(0, error, 0), -0.3, 1.0);
    }


    private void drive() {
        follower.manual(power * direction, 0.0, getHeadingPower());
    }

    private void brake() {
        double headingPower = getHeadingPower();

        double brake = -brakingPower * direction;

        double minBrake = Math.abs(headingPower) + 0.001;

        if (direction > 0) {
            brake = Math.min(brake, -minBrake);
        } else {
            brake = Math.max(brake, minBrake);
        }

        follower.manual(brake, 0, headingPower);
    }

    private void recordBrakeData() {
        double voltage = voltageSensor.getVoltage();
        double duty = state == State.BRAKE ? -brakingPower * direction: power * direction;
        double appliedVoltage = voltage * duty;

        telemetry.addData("timestamp seconds", time);
        telemetry.addData("applied voltage", appliedVoltage);
        telemetry.addData("velocity inches per second", follower.velocity().vx);
        telemetry.addData("position inches", follower.pose().x());
        telemetry.addData("battery voltage", voltage);
        telemetry.addData("duty cycle", duty);
        telemetry.addData("state", state);
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        direction = (iteration % 2 == 0) ? 1 : -1;
        if (iteration < POWERS.length) {
            power = POWERS[iteration];
        }

        if (state != State.DONE) {
            recordBrakeData();
        }

        if (gamepad1.b) {
            follower.stop();
            requestOpModeStop();
            return;
        }

        switch (state) {
            case DRIVE: {
                if ((direction == 1 && follower.pose().x() > (TILES_IN_FRONT_OF_ROBOT - 2) * 24 + 12) ||
                        (direction == -1 && follower.pose().x() < 12)) {
                    startPosition = follower.pose().toVector2D();
                    measuredVelocity = follower.velocity().toVector2D().magnitude();

                    brake();
                    state = State.BRAKE;
                    timer.reset();
                    break;
                }
                drive();
                break;
            }
            case BRAKE: {
                if (follower.velocity().toVector2D().magnitude() > 0.001 && timer.seconds() < MAX_BRAKE_TIME) {
                    brake();
                    break;
                }

                collectTrialData();
                break;
            }
            case WAIT: {
                if (timer.seconds() > IDLE_SECONDS) state = State.DRIVE;
                break;
            }
            case DONE: {}
        }
    }

    @SuppressLint("DefaultLocale")
    public void collectTrialData() {
        Vector2D endPosition = follower.pose().toVector2D();
        double brakingDistance = endPosition.minus(startPosition).magnitude();

        velocityToBrakingDistance.add(new double[]{measuredVelocity, brakingDistance});

        iteration++;

        if (iteration >= POWERS.length) {
            follower.stop();

            double[] coefficients = quadraticFit(velocityToBrakingDistance);

            telemetry.addData("Forward Braking Quadratic", coefficients[1]);
            telemetry.addData("Forward Braking Linear", coefficients[0]);

            telemetry.addLine("Samples:");
            for (int i = 0; i < velocityToBrakingDistance.size(); i++) {
                double[] pair = velocityToBrakingDistance.get(i);
                telemetry.addData("Sample " + i, String.format(" v=%.3f d=%.3f", pair[0], pair[1]));
            }
            telemetry.update();

            state = State.DONE;
        } else {
            state = State.WAIT;
            timer.reset();
        }
    }

    private enum State {
        DRIVE,
        BRAKE,
        WAIT,
        DONE
    }

    private static double[] biasedGradient(
            int count,
            double max,
            double min,
            double bias
    ) {
        if (count < 2) return new double[]{max};

        double[] values = new double[count];

        for (int i = 0; i < count; i++) {
            double t = (double) i / (count - 1);

            double curved = 1 - Math.pow(t, bias);

            values[i] = min + curved * (max - min);
        }

        return values;
    }
}
