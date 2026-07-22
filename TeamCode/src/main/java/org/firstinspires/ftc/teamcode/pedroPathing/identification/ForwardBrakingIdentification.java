package org.firstinspires.ftc.teamcode.pedroPathing.identification;

import android.annotation.SuppressLint;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

public class ForwardBrakingIdentification extends OpMode {
    private static final double[] TEST_POWERS =
            {1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2};
    private static final double BRAKING_POWER = 0.2;

    private static final int DRIVE_TIME_MS = 1000;

    private enum State {
        START_MOVE,
        WAIT_DRIVE_TIME,
        APPLY_BRAKE,
        WAIT_BRAKE_TIME,
        RECORD,
        DONE
    }

    private static class BrakeRecord {
        double timeMs;
        Pose pose;
        double velocity;

        BrakeRecord(double timeMs, Pose pose, double velocity) {
            this.timeMs = timeMs;
            this.pose = pose;
            this.velocity = velocity;
        }
    }

    private State state = State.START_MOVE;

    private final Timer timer = new Timer();

    private int iteration = 0;

    private Vector2D startPosition;
    private double measuredVelocity;

    private final List<double[]> velocityToBrakingDistance = new ArrayList<>();
    private final List<BrakeRecord> brakeData = new ArrayList<>();
    private Follower follower;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(Pose.zero());
        follower.update();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("The robot will move forwards and backwards starting at max speed and slowing down.");
        telemetry.addLine("Make sure you have enough room. Leave at least 4-5 feet.");
        telemetry.addLine("After stopping, kFriction and kBraking will be displayed.");
        telemetry.update();
        follower.update();
    }

    @Override
    public void start() {
        timer.reset();
        follower.update();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        follower.update();

        double direction = (iteration % 2 == 0) ? 1 : -1;

        switch (state) {
            case START_MOVE: {
                if (iteration >= TEST_POWERS.length) {
                    state = State.DONE;
                    break;
                }

                double currentPower = TEST_POWERS[iteration];
                follower.manual(direction * currentPower, 0, 0);

                timer.reset();
                state = State.WAIT_DRIVE_TIME;
                break;
            }

            case WAIT_DRIVE_TIME: {
                if (timer.milliseconds() >= DRIVE_TIME_MS) {
                    measuredVelocity = follower.velocity().toVector2D().magnitude();
                    startPosition = follower.pose().toVector2D();
                    state = State.APPLY_BRAKE;
                }
                break;
            }

            case APPLY_BRAKE: {
                follower.manual(BRAKING_POWER * direction, 0, 0);

                timer.reset();
                state = State.WAIT_BRAKE_TIME;
                break;
            }

            case WAIT_BRAKE_TIME: {
                double t = timer.milliseconds();
                Pose currentPose = follower.pose();
                double currentVelocity = follower.velocity().toVector2D().magnitude();

                brakeData.add(new BrakeRecord(t, currentPose, currentVelocity));


                if (follower.velocity().toVector2D().dot(Vector2D.polar(direction,
                        follower.pose().heading())) <= 0) {
                    state = State.RECORD;
                }
                break;
            }

            case RECORD: {
                Vector2D endPosition = follower.pose().toVector2D();
                double brakingDistance = endPosition.minus(startPosition).magnitude();

                velocityToBrakingDistance.add(new double[]{measuredVelocity, brakingDistance});

                telemetry.addLine(String.format("Test %d: v=%.3f  d=%.3f", iteration, measuredVelocity, brakingDistance));

                iteration++;
                state = State.START_MOVE;

                break;
            }

            case DONE: {

                double[] coefficients = quadraticFit(velocityToBrakingDistance);

                telemetry.addLine("Tuning Complete");
                telemetry.addLine("Braking Profile:");
                telemetry.addData("Forward Quadratic Brake Coefficient", coefficients[1]);
                telemetry.addData("Forward Linear Brake Coefficient", coefficients[0]);
                for (BrakeRecord record : brakeData) {
                    Pose p = record.pose;
                    telemetry.addLine(String.format("t=%.0f ms, x=%.2f, y=%.2f, θ=%.2f, v=%.2f",
                            record.timeMs, p.x(), p.y(),
                            p.heading(),
                            record.velocity));
                }
                break;
            }
        }
        telemetry.update();
    }

    public static double[] quadraticFit(List<double[]> points) {
        double sumX2 = 0, sumX3 = 0, sumX4 = 0;
        double sumXY = 0, sumX2Y = 0;

        for (double[] point : points) {
            double x = point[0];
            double y = point[1];

            sumX2 += x * x;
            sumX3 += x * x * x;
            sumX4 += x * x * x * x;
            sumXY += x * y;
            sumX2Y += x * x * y;
        }

        double[][] matrix = {
                {sumX2, sumX3},
                {sumX3, sumX4}
        };

        double[] constants = {sumXY, sumX2Y};

        return solveLinearSystem(matrix, constants); // returns {b, a}
    }

    private static double[] solveLinearSystem(double[][] A, double[] B) {
        int n = B.length;
        double[] X = new double[n];
        double detA = determinant(A);

        if (Math.abs(detA) < 1e-10) {
            throw new IllegalArgumentException("Matrix is singular or nearly singular");
        }

        for (int i = 0; i < n; i++) {
            double[][] Ai = replaceColumn(A, B, i);
            X[i] = determinant(Ai) / detA;
        }

        return X;
    }

    private static double determinant(double[][] matrix) {
        return matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
    }

    private static double[][] replaceColumn(double[][] matrix, double[] column, int colIndex) {
        double[][] newMatrix = new double[matrix.length][matrix[0].length];

        for (int i = 0; i < matrix.length; i++) {
            System.arraycopy(matrix[i], 0, newMatrix[i], 0, matrix[i].length);
            newMatrix[i][colIndex] = column[i];
        }

        return newMatrix;
    }
}
