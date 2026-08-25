package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Arrays;
import java.util.Stack;

@TeleOp(group = "1")
public class AutomaticOffsetsTuner extends OpMode {
    public static double POWER = 0.3;
    public static double TRIAL_RUNTIME = 3;
    public static int TRIALS = 4;
    private final Timer timer = new Timer();
    private boolean done = false;
    private final Stack<Vector2D> poses = new Stack<>();
    private final Stack<Vector2D> offsetResults = new Stack<>();
    private Vector2D offsets;
    private int trialsCompleted;

    private static class Circle {
        Vector2D center;
        double radius;

        Circle(Vector2D center, double radius) {
            this.center = center;
            this.radius = radius;
        }
    }

    private Follower follower;

    @Override
    public void init() {
        Constants.localizerConfig.xPodOffset.set(0.0);
        Constants.localizerConfig.yPodOffset.set(0.0);
        follower = Constants.create(hardwareMap);
        follower.setPose(Pose.zero());
        follower.update();
    }

    @Override
    public void start() {
        follower.setPose(Pose.zero());
        timer.reset();
        follower.manual(0, 0, POWER);
        follower.update();
    }

    /**
     * This initializes the PoseUpdater as well as the Panels telemetry.
     */
    @Override
    public void init_loop() {
        telemetry.addLine("This will turn continuously in place for " + TRIAL_RUNTIME * TRIALS + " seconds.");
        telemetry.addLine("Make sure you have enough room.");
        telemetry.update();
        follower.update();
    }

    /**
     * This updates the robot's pose estimate, and updates the Panels telemetry with the
     * calculated offsets and draws the robot.
     */
    @Override
    public void loop() {
        follower.update();

        if (gamepad1.bWasPressed()) {
            follower.manual(0, 0, 0);
            requestOpModeStop();
        }

        if (!done) {
            poses.push(follower.pose().toVector2D());

            if (timer.seconds() >= TRIAL_RUNTIME) {
                offsetResults.push(fitCircle(poses.toArray(new Vector2D[0])));
                trialsCompleted++;

                if (trialsCompleted >= TRIALS) {
                    done = true;
                    offsets = offsetResults
                            .stream()
                            .reduce(Vector2D::plus)
                            .orElse(Vector2D.zero())
                            .div(TRIALS);
                } else {
                    follower.setPose(Pose.zero());
                    poses.clear();
                    timer.reset();
                }
            } else {
                follower.manual(0, 0, POWER);
            }
        } else {
            follower.manual(0, 0, 0);
            telemetry.addLine("The following values are the offsets in inches that should be applied to your localizer.");
            telemetry.addLine("xPodOffset: " + offsets.y());
            telemetry.addLine("yPodOffset: " + offsets.x());
            telemetry.update();
        }
    }

    private Vector2D fitCircle(Vector2D[] points) {
        if (points.length >= 30) {
            points = Arrays.copyOfRange(points, 20, points.length);
        }

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
