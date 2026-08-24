package org.firstinspires.ftc.teamcode.pedroPathing.identification;

import static com.pedropathing.utils.Angle.normalizeSigned;

import android.annotation.SuppressLint;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Jacob Ophoven - 12649 Code Blooded
 * @version 8/11/2026
 */
@TeleOp(group = "2")
public class StrafeBrakingIdentification extends OpMode {
    private static double[] POWERS;

    public static int trials = 12;
    public static double maxPower = 1;
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
        telemetry.addLine("It will strafe at different powers first to the left, then to the right, measuring braking distance while correcting its heading.");
        telemetry.addLine("Make sure you have enough room.");
        telemetry.addLine("After stopping, the lateral linear and quadratic braking coefficients will be displayed.");
        telemetry.update();
        follower.update();
    }

    private double getHeadingPower() {
        return Constants.foresightConfig.headingController.get()
                .calculate(0, normalizeSigned(-follower.pose().heading()), follower.velocity().omega); //TODO: update for ForesightV3
    }

    private void drive() {
        follower.manual(0.0, power * direction, getHeadingPower());
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

        follower.manual(0, brake, headingPower);
    }

    private void recordBrakeData() {
        double voltage = voltageSensor.getVoltage();
        double duty = state == State.BRAKE ? -brakingPower * direction: power * direction;
        double appliedVoltage = voltage * duty;

        telemetry.addData("timestamp seconds", time);
        telemetry.addData("applied voltage", appliedVoltage);
        telemetry.addData("velocity inches per second", follower.velocity().vy);
        telemetry.addData("position inches", follower.pose().y());
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
                if ((direction == 1 && follower.pose().y() > (TILES_IN_FRONT_OF_ROBOT - 2) * 24 + 12) ||
                        (direction == -1 && follower.pose().y() < 12)) {
                    startPosition = follower.pose().toVector2D();
                    measuredVelocity = follower.velocity().toVector2D().magnitude();

                    brake();
                    state = State.BRAKE;
                    break;
                }
                drive();
                break;
            }
            case BRAKE: {
                if (follower.velocity().toVector2D().magnitude() > 0.001) {
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

            telemetry.addData("Lateral Braking Quadratic", coefficients[1]);
            telemetry.addData("Lateral Braking Linear", coefficients[0]);

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

    public static double[] quadraticFit(List<double[]> samples) {
        double s11 = 0.0;
        double s12 = 0.0;
        double s22 = 0.0;

        double t1 = 0.0;
        double t2 = 0.0;

        for (double[] sample : samples) {
            double x1 = sample[0];
            double d = sample[1];

            double x2 = x1 * x1;

            s11 += x1 * x1;
            s12 += x1 * x2;
            s22 += x2 * x2;

            t1 += x1 * d;
            t2 += x2 * d;
        }

        double det = s11 * s22 - s12 * s12;
        if (Math.abs(det) < 1e-12) {
            throw new IllegalArgumentException("Regression matrix is singular.");
        }

        double b = (t1 * s22 - t2 * s12) / det;
        double a = (s11 * t2 - s12 * t1) / det;

        return new double[]{b, a};
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
