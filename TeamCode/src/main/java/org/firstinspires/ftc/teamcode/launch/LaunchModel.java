package org.firstinspires.ftc.teamcode.launch;

/**
 * Ball flight model for choosing flywheel RPM, ported from the BIOBUZZ sim (BallFlight, CellTarget and
 * main.gd's RPM solver), which itself matches the matplotlib sim.
 *
 * The flight is worked out in the vertical plane of the shot: gravity, drag and topspin lift, integrated
 * with the same 1 ms Euler-Cromer steps. A shot scores when the ball passes inwards through the front and
 * back faces of the raised CELL's opening with the whole ball inside the opening outline. The best RPM is
 * the middle of the widest range of RPMs that score, the furthest from a miss for speed errors either way.
 *
 * Simplifications: the shot is treated as head-on to the CELL, and the ball is assumed to cross the
 * opening on its centre line (an off-centre launcher crosses up to ~0.15 m to one side, well inside the
 * 0.51 m wide opening).
 *
 * Distances in metres, speeds in m/s.
 */
public class LaunchModel {
    // Physics (same values as the sim)
    private static final double G = 9.81;
    private static final double RHO = 1.225;
    private static final double DT = 0.001;
    private static final double MAX_TIME = 3.0;
    public static double dragCoefficient = 0.45;
    public static double liftCoefficient = 0.5;

    public static final double FLYWHEEL_DIAMETER = 0.090;
    /** Ball exit speed / flywheel surface speed, measured on the 60 degree launcher. */
    public static double exitSpeedRatio = 0.406;

    // RPM search (same as the sim)
    private static final double SOLVE_STEP_RPM = 25.0;
    private static final double SOLVE_MIN_RPM = 500.0;
    public static final double MAX_FLYWHEEL_RPM = 4500.0;

    /** Launcher geometry from the robot CAD. Robot frame: forward, left, up from the floor. */
    public enum Launcher {
        POLLEN(0.02268, -0.15409, 0.19726, 0.0249, 0.0711),
        NECTAR(0.01668, 0.12062, 0.21726, 0.0413, 0.0914);

        public final double axleForward, axleLeft, axleHeight;
        public final double ballMass, ballDiameter;
        public final double angleDeg = 76.0;
        /** Straight section of hood after the arc. */
        public final double runout = 0.0136;

        Launcher(double axleForward, double axleLeft, double axleHeight, double ballMass, double ballDiameter) {
            this.axleForward = axleForward;
            this.axleLeft = axleLeft;
            this.axleHeight = axleHeight;
            this.ballMass = ballMass;
            this.ballDiameter = ballDiameter;
        }
    }

    // Raised CELL opening, from the field CAD
    /** Height of the point LimelightCamera reports as the open face's centre. */
    public static final double FACE_HEIGHT = 1.4804;
    /** The opening faces this far above horizontal. */
    private static final double FACE_ELEVATION = Math.toRadians(29.317);
    /** LimelightCamera's target point is this far in front of the opening's centre, along its normal. */
    private static final double CAMERA_POINT_IN_FRONT = 0.0193;
    /** Front and back faces of the opening, either side of its centre along the normal. */
    private static final double FACE_HALF_DEPTH = 0.0256;
    /** Opening outline in the face (u right, v up the face), anticlockwise, about the opening centre. */
    private static final double[][] OPENING = {
            {-0.25404, 0.04043}, {-0.25401, -0.11833}, {-0.25333, -0.12412}, {-0.25151, -0.12932},
            {-0.24855, -0.13404}, {-0.24448, -0.13816}, {-0.2396, -0.14123}, {-0.23431, -0.14308},
            {-0.2286, -0.14374}, {0.2286, -0.14374}, {0.23437, -0.14306}, {0.23958, -0.14124},
            {0.24423, -0.13832}, {0.24839, -0.13426}, {0.25152, -0.1293}, {0.25331, -0.12418},
            {0.25401, -0.11833}, {0.25401, -0.11374}, {0.25404, 0.04043}, {0.25363, 0.04463},
            {0.24589, 0.05888}, {0.2423, 0.06188}, {0.2149, 0.07939}, {0.01371, 0.20796},
            {0.00718, 0.21086}, {-0.00358, 0.2117}, {-0.01372, 0.20796}, {-0.21664, 0.07827},
            {-0.2423, 0.06187}, {-0.24562, 0.05926}, {-0.25346, 0.04505}
    };
    private static final double[][] EDGE_NORMALS = new double[OPENING.length][];

    static {
        for (int k = 0; k < OPENING.length; k++) {
            double[] a = OPENING[k];
            double[] b = OPENING[(k + 1) % OPENING.length];
            double ex = b[0] - a[0], ey = b[1] - a[1];
            double length = Math.hypot(ex, ey);
            EDGE_NORMALS[k] = new double[] {-ey / length, ex / length};  // inward, as the outline is anticlockwise
        }
    }

    public static double exitSpeed(double rpm) {
        return exitSpeedRatio * rpm / 60.0 * Math.PI * FLYWHEEL_DIAMETER;
    }

    public static double rpmForSpeed(double speed) {
        return speed / (exitSpeedRatio * Math.PI * FLYWHEEL_DIAMETER) * 60.0;
    }

    /**
     * Does a shot at rpm score, from a launcher whose axle is axleDistance (horizontally) from
     * LimelightCamera's target point at faceHeight?
     */
    public static boolean scores(Launcher launcher, double rpm, double axleDistance, double faceHeight) {
        double radius = launcher.ballDiameter / 2.0;
        double area = Math.PI * radius * radius;
        double kDrag = 0.5 * RHO * dragCoefficient * area / launcher.ballMass;
        double kLift = 0.5 * RHO * liftCoefficient * area / launcher.ballMass;

        // Opening frame in the shot plane (s towards the CELL, y up): n out of the face, w up the face
        double nS = -Math.cos(FACE_ELEVATION), nY = Math.sin(FACE_ELEVATION);
        double wS = Math.sin(FACE_ELEVATION), wY = Math.cos(FACE_ELEVATION);
        double centreS = axleDistance - CAMERA_POINT_IN_FRONT * nS;
        double centreY = faceHeight - CAMERA_POINT_IN_FRONT * nY;
        double frontS = centreS + FACE_HALF_DEPTH * nS, frontY = centreY + FACE_HALF_DEPTH * nY;
        double backS = centreS - FACE_HALF_DEPTH * nS, backY = centreY - FACE_HALF_DEPTH * nY;

        // Launch: the ball leaves the hood (wheel radius + ball radius) from the axle at the launch angle
        // round from straight down, then along a straight runout
        double angle = Math.toRadians(launcher.angleDeg);
        double reach = FLYWHEEL_DIAMETER / 2.0 + radius;
        double s = reach * Math.sin(angle) + launcher.runout * Math.cos(angle);
        double y = launcher.axleHeight - reach * Math.cos(angle) + launcher.runout * Math.sin(angle);
        double speed0 = exitSpeed(rpm);
        double vs = speed0 * Math.cos(angle);
        double vy = speed0 * Math.sin(angle);

        int state = 0;  // 0: outside, 1: through the front face, 2: scored
        int steps = (int) (MAX_TIME / DT);
        for (int i = 0; i < steps; i++) {
            double speed = Math.hypot(vs, vy);
            // Drag against the motion; topspin lift along v x spin axis, which is (vy, -vs) here
            double as = -kDrag * speed * vs + kLift * speed * vy;
            double ay = -G - kDrag * speed * vy - kLift * speed * vs;
            vs += as * DT;
            vy += ay * DT;
            double sNew = s + vs * DT;
            double yNew = y + vy * DT;

            double fa = (s - frontS) * nS + (y - frontY) * nY;
            double fb = (sNew - frontS) * nS + (yNew - frontY) * nY;
            if (state == 0) {
                if (fa > 0 && fb <= 0) {
                    double f = fa / (fa - fb);
                    if (fits(s + (sNew - s) * f, y + (yNew - y) * f, vs, vy, centreS, centreY, nS, nY, wS, wY, radius)) {
                        state = 1;
                    }
                }
            } else if (fb > 0) {
                state = 0;  // came back out of the front
            } else {
                double ba = (s - backS) * nS + (y - backY) * nY;
                double bb = (sNew - backS) * nS + (yNew - backY) * nY;
                if (ba > 0 && bb <= 0) {
                    double f = ba / (ba - bb);
                    return fits(s + (sNew - s) * f, y + (yNew - y) * f, vs, vy, centreS, centreY, nS, nY, wS, wY, radius);
                }
            }

            s = sNew;
            y = yNew;
            if (y < radius || s > centreS + 1.0) return false;
        }
        return false;
    }

    /** Is the whole ball inside the opening outline as its centre crosses a face at (s, y)? */
    private static boolean fits(double s, double y, double vs, double vy, double centreS, double centreY,
                                double nS, double nY, double wS, double wY, double radius) {
        double u = 0.0;  // crossing on the opening's centre line
        double w = (s - centreS) * wS + (y - centreY) * wY;
        double speed = Math.hypot(vs, vy);
        double cosIncidence = Math.abs((vs * nS + vy * nY) / speed);
        if (cosIncidence < 1e-3) return false;
        // Direction of travel projected onto the face, and the perpendicular in the face
        double e2u = 0.0, e2w = Math.signum((vs * wS + vy * wY) / speed);
        if (e2w == 0) e2w = 1.0;
        double e1u = -e2w, e1w = e2u;
        double a1 = radius;
        double a2 = radius / cosIncidence;
        for (int k = 0; k < OPENING.length; k++) {
            double[] n = EDGE_NORMALS[k];
            double clearance = (u - OPENING[k][0]) * n[0] + (w - OPENING[k][1]) * n[1];
            double need = Math.hypot(a1 * (n[0] * e1u + n[1] * e1w), a2 * (n[0] * e2u + n[1] * e2w));
            if (clearance < need) return false;
        }
        return true;
    }

    /**
     * The middle of the widest range of RPMs that score from axleDistance, or NaN if none do.
     * Coarse steps of 25 rpm, then each edge of the range is refined to about 1 rpm.
     */
    public static double solveRPM(Launcher launcher, double axleDistance, double faceHeight) {
        int steps = (int) ((MAX_FLYWHEEL_RPM - SOLVE_MIN_RPM) / SOLVE_STEP_RPM);
        int bestStart = -1, bestEnd = -1, runStart = -1;
        for (int k = 0; k <= steps + 1; k++) {
            boolean hit = k <= steps && scores(launcher, SOLVE_MIN_RPM + k * SOLVE_STEP_RPM, axleDistance, faceHeight);
            if (hit && runStart < 0) {
                runStart = k;
            } else if (!hit && runStart >= 0) {
                if (bestStart < 0 || k - 1 - runStart > bestEnd - bestStart) {
                    bestStart = runStart;
                    bestEnd = k - 1;
                }
                runStart = -1;
            }
        }
        if (bestStart < 0) return Double.NaN;

        double low = SOLVE_MIN_RPM + bestStart * SOLVE_STEP_RPM;
        double high = SOLVE_MIN_RPM + bestEnd * SOLVE_STEP_RPM;
        if (bestStart > 0) low = edge(launcher, low - SOLVE_STEP_RPM, low, axleDistance, faceHeight);
        if (bestEnd < steps) high = edge(launcher, high + SOLVE_STEP_RPM, high, axleDistance, faceHeight);
        return (low + high) / 2.0;
    }

    /** Bisect between a missing and a scoring RPM to find the edge of the scoring range. */
    private static double edge(Launcher launcher, double missing, double scoring, double axleDistance, double faceHeight) {
        for (int i = 0; i < 5; i++) {
            double mid = (missing + scoring) / 2.0;
            if (scores(launcher, mid, axleDistance, faceHeight)) {
                scoring = mid;
            } else {
                missing = mid;
            }
        }
        return scoring;
    }
}
