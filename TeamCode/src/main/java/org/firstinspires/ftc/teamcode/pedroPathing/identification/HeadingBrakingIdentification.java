package org.firstinspires.ftc.teamcode.pedroPathing.identification;

import static com.pedropathing.utils.Utils.quadraticFit;

import android.annotation.SuppressLint;

import com.pedropathing.follower.Follower;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Jacob Ophoven - 12649 Code Blooded
 * @author Havish Sripada - 12808 RevAmped Robotics
 * @version 8/22/2026
 */
@TeleOp(group = "2")
public class HeadingBrakingIdentification extends OpMode {
    private static double[] POWERS;
    public static double MAX_BRAKE_TIME = 4; //seconds, the robot shouldn't take longer than this to brake

    public static int trials = 12;
    public static double maxPower = 1;
    public static double minPower = 0.2;
    public static double bias = 1.5; // how much it favors doing trials with higher powers
    public static double brakingPower = 0.001;

    private final ElapsedTime timer = new ElapsedTime();

    private final List<double[]> velocityToBrakingDistance = new ArrayList<>();
    private State state = State.DRIVE;
    private int iteration = 0;
    private int direction;
    private double power;

    private double startHeading;
    private double measuredVelocity;
    private double totalHeading;
    private double previousHeading;

    private Follower follower;
    private VoltageSensor voltageSensor;

    @Override
    public void init() {
        POWERS = biasedGradient(trials, maxPower, minPower, bias);

        follower = Constants.create(hardwareMap);
        voltageSensor = hardwareMap.getAll(VoltageSensor.class).iterator().next();

        follower.update();
        recordBrakeData();

        previousHeading = follower.pose().heading();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("The robot will turn back at forth at various speed levels.");
        telemetry.addLine("Make sure you have enough room.");
        telemetry.addLine("After it is finished, the heading linear and quadratic braking coefficients will be displayed.");
        telemetry.update();
        follower.update();
    }

    private void drive() {
        follower.manual(0.0, 0.0, power * direction);
    }

    private void brake() {
        follower.manual(0, 0, -brakingPower * direction);
    }

    private void recordBrakeData() {
        double voltage = voltageSensor.getVoltage();
        double duty = state == State.BRAKE ? -brakingPower * direction: power * direction;
        double appliedVoltage = voltage * duty;

        telemetry.addData("timestamp seconds", time);
        telemetry.addData("applied voltage", appliedVoltage);
        telemetry.addData("velocity radians per second", follower.velocity().omega);
        telemetry.addData("battery voltage", voltage);
        telemetry.addData("duty cycle", duty);
        telemetry.addData("state", state);
        telemetry.update();
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        follower.update();
        double currentHeading = follower.pose().heading();
        totalHeading += Angle.normalizeSigned(currentHeading - previousHeading);
        previousHeading = currentHeading;

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
                if (timer.seconds() > 2) {
                    startHeading = totalHeading;
                    measuredVelocity = Math.abs(follower.velocity().omega);

                    brake();
                    state = State.BRAKE;
                    timer.reset();
                    break;
                }
                drive();
                break;
            }
            case BRAKE: {
                if (Math.abs(follower.velocity().omega) > 0.001 && timer.seconds() < MAX_BRAKE_TIME) {
                    brake();
                    break;
                }

                collectTrialData();
                break;
            }
            case DONE: {}
        }
    }

    @SuppressLint("DefaultLocale")
    public void collectTrialData() {
        double endHeading = totalHeading;
        double brakingDistance = Math.abs(endHeading - startHeading);

        velocityToBrakingDistance.add(new double[]{measuredVelocity, brakingDistance});

        iteration++;

        if (iteration >= POWERS.length) {
            follower.stop();

            double[] coefficients = quadraticFit(velocityToBrakingDistance);

            telemetry.addLine("Heading Tuning Complete");
            telemetry.addData("quadratic", coefficients[1]);
            telemetry.addData("linear", coefficients[0]);

            telemetry.addLine("Samples:");
            for (int i = 0; i < velocityToBrakingDistance.size(); i++) {
                double[] pair = velocityToBrakingDistance.get(i);
                telemetry.addData("Sample " + i, String.format(" v=%.3f d=%.3f", pair[0], pair[1]));
            }
            telemetry.update();

            state = State.DONE;
        } else {
            timer.reset();
            state = State.DRIVE;
        }
    }

    private enum State {
        DRIVE,
        BRAKE,
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
