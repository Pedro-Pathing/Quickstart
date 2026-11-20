package org.firstinspires.ftc.teamcode.opmodes.TeleOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.FieldImage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "Odometry Drift Test")
public class OdometryDriftTest extends LinearOpMode {
    public static double START_X_IN = 132;
    public static double START_Y_IN = 133;
    public static double START_HEADING_DEG = -90;
    public static boolean FIELD_CENTRIC = false;
    public static double DRIVE_SPEED = 0.6;
    public static double TRAIL_MIN_STEP_IN = 1;
    public static int TRAIL_MAX_POINTS = 1000;
    public static double ROBOT_RADIUS_IN = 9;

    private final List<double[]> trail = new ArrayList<>();
    private double distanceDriven = 0;
    private double lastX, lastY;
    private Pose measured = null;
    private double measuredAtDistance = 0;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Follower follower = Constants.createFollower(hardwareMap);
        Pose start = startPose();
        follower.setPose(start);
        reset(start);

        while (opModeInInit()) {
            start = startPose();
            follower.setPose(start);
            follower.update();
            reset(start);
            telemetry.addLine("Place robot on the marked start spot, matching START_* on Dashboard.");
            telemetry.addData("Start", fmt(start));
            telemetry.update();
            send(dashboard, follower.pose(), start);
        }

        boolean prevA = false, prevX = false, prevB = false;
        while (opModeIsActive()) {
            double headingOffset = FIELD_CENTRIC ? follower.pose().heading() : 0;
            follower.manual(ManualDrive.fieldCentric(
                    gamepad1.left_stick_y * DRIVE_SPEED,
                    gamepad1.left_stick_x * DRIVE_SPEED,
                    -gamepad1.right_stick_x * DRIVE_SPEED,
                    headingOffset
            ));
            follower.update();
            Pose pose = follower.pose();
            track(pose);

            if (gamepad1.a && !prevA) {
                measured = pose;
                measuredAtDistance = distanceDriven;
            }
            if (gamepad1.x && !prevX) {
                follower.setPose(start);
                reset(start);
                pose = start;
            }
            if (gamepad1.b && !prevB) {
                trail.clear();
                trail.add(new double[]{pose.x(), pose.y()});
            }
            prevA = gamepad1.a;
            prevX = gamepad1.x;
            prevB = gamepad1.b;

            telemetry.addLine("A = measure drift (robot back on start spot), X = reset pose, B = clear trail");
            telemetry.addData("Pose", fmt(pose));
            telemetry.addData("Distance driven in", "%.1f", distanceDriven);
            addDrift(telemetry, start);
            telemetry.update();
            send(dashboard, pose, start);
        }
    }

    private Pose startPose() {
        return new Pose(START_X_IN, START_Y_IN, Math.toRadians(START_HEADING_DEG));
    }

    private void reset(Pose start) {
        trail.clear();
        trail.add(new double[]{start.x(), start.y()});
        distanceDriven = 0;
        lastX = start.x();
        lastY = start.y();
        measured = null;
    }

    private void track(Pose pose) {
        double step = Math.hypot(pose.x() - lastX, pose.y() - lastY);
        if (step < TRAIL_MIN_STEP_IN) return;
        distanceDriven += step;
        lastX = pose.x();
        lastY = pose.y();
        trail.add(new double[]{pose.x(), pose.y()});
        if (trail.size() > TRAIL_MAX_POINTS) trail.remove(0);
    }

    private double[] drift(Pose start) {
        double dx = measured.x() - start.x();
        double dy = measured.y() - start.y();
        double dh = Math.toDegrees(Turret.wrap(measured.heading() - start.heading()));
        double dist = Math.hypot(dx, dy);
        double per100 = measuredAtDistance > 0 ? dist / measuredAtDistance * 100 : 0;
        return new double[]{dx, dy, dist, dh, per100};
    }

    private void addDrift(org.firstinspires.ftc.robotcore.external.Telemetry t, Pose start) {
        if (measured == null) {
            t.addData("Drift", "drive back to the start spot and press A");
            return;
        }
        double[] d = drift(start);
        t.addData("Drift x / y in", "%.2f / %.2f", d[0], d[1]);
        t.addData("Drift distance in", "%.2f", d[2]);
        t.addData("Drift heading deg", "%.2f", d[3]);
        t.addData("Drift per 100 in driven", "%.2f in (over %.0f in)", d[4], measuredAtDistance);
    }

    private void send(FtcDashboard dashboard, Pose pose, Pose start) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("x", pose.x());
        packet.put("y", pose.y());
        packet.put("heading deg", Math.toDegrees(pose.heading()));
        packet.put("distance driven in", distanceDriven);
        if (measured != null) {
            double[] d = drift(start);
            packet.put("drift x in", d[0]);
            packet.put("drift y in", d[1]);
            packet.put("drift dist in", d[2]);
            packet.put("drift heading deg", d[3]);
            packet.put("drift per 100 in", d[4]);
        }

        Canvas c = packet.fieldOverlay();
        FieldImage.drawPedro(c);

        c.setStrokeWidth(1);
        c.setStroke("#888888");
        drawMarker(c, Field.RED_CELL_TOP);
        drawMarker(c, Field.RED_CELL_BOTTOM);
        drawMarker(c, Field.BLUE_CELL_TOP);
        drawMarker(c, Field.BLUE_CELL_BOTTOM);

        if (trail.size() > 1) {
            double[] xs = new double[trail.size()];
            double[] ys = new double[trail.size()];
            for (int i = 0; i < trail.size(); i++) {
                xs[i] = trail.get(i)[0];
                ys[i] = trail.get(i)[1];
            }
            c.setStroke("#4CAF50");
            c.strokePolyline(xs, ys);
        }

        c.setStroke("#FFC107");
        c.setStrokeWidth(2);
        drawRobot(c, start);

        if (measured != null) {
            c.setStroke("#F44336");
            c.strokeLine(start.x(), start.y(), measured.x(), measured.y());
            drawRobot(c, measured);
        }

        c.setStroke("#2196F3");
        drawRobot(c, pose);

        dashboard.sendTelemetryPacket(packet);
    }

    private void drawRobot(Canvas c, Pose p) {
        double x = p.x(), y = p.y();
        c.strokeCircle(x, y, ROBOT_RADIUS_IN);
        c.strokeLine(x, y,
                x + ROBOT_RADIUS_IN * Math.cos(p.heading()),
                y + ROBOT_RADIUS_IN * Math.sin(p.heading()));
    }

    private void drawMarker(Canvas c, Pose p) {
        c.strokeCircle(p.x(), p.y(), 3);
    }

    private static String fmt(Pose p) {
        return String.format("x %.2f  y %.2f  h %.1f°", p.x(), p.y(), Math.toDegrees(p.heading()));
    }
}
