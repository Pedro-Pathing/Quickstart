package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.localizers.CustomIMU;
import com.pedropathing.revhub.localizers.ThreeWheelConfig;
import com.pedropathing.revhub.localizers.TwoWheelConfig;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.pedropathing.utils.Timer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Arrays;
import java.util.Stack;
import java.util.function.BiFunction;

public class ThreeWheelAutomaticOffsetsTuner extends OpMode {
    public static double POWER = 0.5;
    public static double TRIAL_RUNTIME = 6;
    public static int TRIALS = 5;
    private final Timer timer = new Timer();
    private final Stack<Vector2D> poses = new Stack<>();
    private Vector2D offsetsLeft;
    private Vector2D offsetsRight;
    private int trialsCompleted;
    private final Stack<Vector2D> offsetResults = new Stack<>();

    private static class Circle {
        Vector2D center;
        double radius;

        Circle(Vector2D center, double radius) {
            this.center = center;
            this.radius = radius;
        }
    }

    private final BiFunction<HardwareMap, Localizer, Follower> followerCreator;
    private final ThreeWheelConfig config;

    private Follower follower;

    private static class BlankIMU implements CustomIMU {
        @Override
        public void initialize(HardwareMap hardwareMap, String hardwareMapName, RevHubOrientationOnRobot hubOrientation) {}

        @Override
        public double getHeading() {
            return 0;
        }

        @Override
        public void resetYaw() {}
    }

    private final RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
    );

    private TwoWheelConfig configLeft;
    private TwoWheelConfig configRight;
    
    private enum State {
        LEFT_POD,
        RIGHT_POD,
        IDLE
    }
    
    private State state = State.LEFT_POD;

    public ThreeWheelAutomaticOffsetsTuner(BiFunction<HardwareMap, Localizer, Follower> followerCreator, ThreeWheelConfig config) {
        this.followerCreator = followerCreator;
        this.config = config;
    }

    @Override
    public void init() {
        configLeft = new TwoWheelConfig(c -> {
            c.xPodName.set(config.leftEncoderName.get());
            c.yPodName.set(config.strafeEncoderName.get());
            c.forwardTicksToInches.set(config.forwardTicksToInches.get());
            c.strafeTicksToInches.set(config.strafeTicksToInches.get());
            c.xPodDirection.set(config.leftEncoderDirection.get());
            c.yPodDirection.set(config.strafeEncoderDirection.get());
            c.imu.set(new BlankIMU());
            c.imuName.set("hi");
            c.imuOrientation.set(orientation);
            c.xPodOffset.set(0.0);
            c.yPodOffset.set(0.0);
        });

        configRight = new TwoWheelConfig(c -> {
            c.xPodName.set(config.leftEncoderName.get());
            c.yPodName.set(config.strafeEncoderName.get());
            c.forwardTicksToInches.set(config.forwardTicksToInches.get());
            c.strafeTicksToInches.set(config.strafeTicksToInches.get());
            c.xPodDirection.set(config.leftEncoderDirection.get());
            c.yPodDirection.set(config.strafeEncoderDirection.get());
            c.imu.set(new BlankIMU());
            c.imuName.set("hi");
            c.imuOrientation.set(orientation);
            c.xPodOffset.set(0.0);
            c.yPodOffset.set(0.0);
        });
        
        initLeftPod();
    }

    @Override
    public void start() {
        beginProcedure();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("This will turn continuously in place for " + TRIAL_RUNTIME + " seconds.");
        telemetry.addLine("Make sure you have enough room.");
        telemetry.update();
        follower.update();
    }

    private void initLeftPod() {
        follower = followerCreator.apply(hardwareMap, new TwoWheelLocalizer(hardwareMap, configLeft));
        follower.setPose(Pose.zero());
        follower.update();
    }

    private void initRightPod() {
        follower = followerCreator.apply(hardwareMap, new TwoWheelLocalizer(hardwareMap, configRight));
        follower.setPose(Pose.zero());
        follower.update();
    }

    private void beginProcedure() {
        follower.setPose(Pose.zero());
        timer.reset();
        follower.manual(0, 0, POWER);
        poses.clear();
        follower.update();
        offsetResults.clear();
        trialsCompleted = 0;
    }

    private void executeProcedure() {
        follower.update();

        if (gamepad1.bWasPressed()) {
            follower.manual(0, 0, 0);
            requestOpModeStop();
        }

        poses.push(follower.pose().toVector2D());

        if (timer.seconds() >= TRIAL_RUNTIME / 2.0) {
            offsetResults.push(fitCircle(poses.toArray(new Vector2D[0])));
            trialsCompleted++;

            if (trialsCompleted >= TRIALS) {
                if (state.equals(State.LEFT_POD)) {
                    offsetsLeft = offsetResults
                            .stream()
                            .reduce(Vector2D::plus)
                            .orElse(Vector2D.zero())
                            .div(TRIALS);

                    initRightPod();
                    beginProcedure();
                    state = State.RIGHT_POD;
                } else {
                    state = State.IDLE;

                    follower.manual(0, 0, 0);

                    offsetsRight = offsetResults
                            .stream()
                            .reduce(Vector2D::plus)
                            .orElse(Vector2D.zero())
                            .div(TRIALS);
                }
            } else {
                follower.setPose(Pose.zero());
                poses.clear();
                timer.reset();
            }
        }
    }

    private void printResults() {
        telemetry.addLine("The following values are the offsets in inches that should be applied to your localizer.");
        telemetry.addLine("leftPodOffset: " + offsetsLeft.y());
        telemetry.addLine("rightPodOffset: " + offsetsRight.y());
        telemetry.addLine("strafePodOffset: " + (offsetsLeft.x() + offsetsRight.x()) / 2);
        telemetry.update();
    }

    @Override
    public void loop() {
        if (state.equals(State.IDLE)) printResults();
        else executeProcedure();
    }

    private Vector2D fitCircle(Vector2D[] points) {
        points = Arrays.copyOfRange(points, 20, points.length);
        Circle circle = taubin(points);
        circle = gaussNewton(points, circle);
        return circle.center.times(-1);
    }

    private static Circle taubin(Vector2D[] pts) {
        int n = pts.length;

        double mx = 0, my = 0;
        for (Vector2D p : pts) {
            mx += p.x();
            my += p.y();
        }
        mx /= n;
        my /= n;

        double Mxx = 0, Myy = 0, Mxy = 0, Mxz = 0, Myz = 0, Mzz = 0;
        for (Vector2D p : pts) {
            double x = p.x() - mx, y = p.y() - my;
            double z = x * x + y * y;
            Mxx += x * x;
            Myy += y * y;
            Mxy += x * y;
            Mxz += x * z;
            Myz += y * z;
            Mzz += z * z;
        }
        Mxx /= n;
        Myy /= n;
        Mxy /= n;
        Mxz /= n;
        Myz /= n;
        Mzz /= n;

        double Mz = Mxx + Myy;
        double CovXy = Mxx * Myy - Mxy * Mxy;
        double VarZ = Mzz - Mz * Mz;

        double A3 = 4 * Mz;
        double A2 = -3 * Mz * Mz - Mzz;
        double A1 = VarZ * Mz + 4 * CovXy * Mz - Mxz * Mxz - Myz * Myz;
        double A0 = Mxz * (Mxz * Myy - Myz * Mxy) + Myz * (Myz * Mxx - Mxz * Mxy) - VarZ * CovXy;
        double A22 = A2 + A2;
        double A33 = A3 + A3 + A3;

        double x = 0, y = A0;
        for (int iter = 0; iter < 99; iter++) {
            double dy = A1 + x * (A22 + A33 * x);
            double xnew = x - y / dy;
            if (xnew == x || !Double.isFinite(xnew)) break;
            double ynew = A0 + xnew * (A1 + xnew * (A2 + xnew * A3));
            if (Math.abs(ynew) >= Math.abs(y)) break;
            x = xnew;
            y = ynew;
        }

        double det = x * x - x * Mz + CovXy;
        double cxc = (Mxz * (Myy - x) - Myz * Mxy) / det / 2.0;
        double cyc = (Myz * (Mxx - x) - Mxz * Mxy) / det / 2.0;
        double r = Math.sqrt(cxc * cxc + cyc * cyc + Mz);

        Vector2D center = Vector2D.cartesian(cxc + mx, cyc + my);
        return new Circle(center, r);
    }

    private static Circle gaussNewton(Vector2D[] pts, Circle init) {
        double a = init.center.x();
        double b = init.center.y();
        double r = init.radius;
        int n = pts.length;

        for (int iter = 0; iter < 200; iter++) {
            double[] res = new double[n];
            double[] da = new double[n];
            double[] db = new double[n];
            double[] dr = new double[n];

            for (int i = 0; i < n; i++) {
                double dx = pts[i].x() - a;
                double dy = pts[i].y() - b;
                double dist = Math.sqrt(dx * dx + dy * dy);
                if (dist < 1e-4) continue;
                res[i] = dist - r;
                da[i] = -dx / dist;
                db[i] = -dy / dist;
                dr[i] = -1.0;
            }

            double[][] jCols = {da, db, dr};

            double[][] JtJ_data = new double[3][3];
            double[][] Jtf_data = new double[3][1];

            for (int row = 0; row < 3; row++) {
                double jtf = 0;
                for (int i = 0; i < n; i++) jtf += jCols[row][i] * res[i];
                Jtf_data[row][0] = jtf;

                for (int col = 0; col < 3; col++) {
                    double jtj = 0;
                    for (int i = 0; i < n; i++) jtj += jCols[row][i] * jCols[col][i];
                    JtJ_data[row][col] = jtj;
                }
            }

            Matrix JtJ = new Matrix(JtJ_data);
            Matrix Jtf = new Matrix(Jtf_data);

            double gradNorm = 0;
            for (int i = 0; i < 3; i++) gradNorm += Jtf.get(i, 0) * Jtf.get(i, 0);
            if (Math.sqrt(gradNorm) * 2 < 1e-12) break;

            Matrix invJtJ = Matrix.inverse3x3(JtJ);
            Matrix delta = invJtJ.times(Jtf);

            a -= delta.get(0, 0);
            b -= delta.get(1, 0);
            r -= delta.get(2, 0);
        }

        Vector2D center = Vector2D.cartesian(a, b);
        return new Circle(center, r);
    }
}
