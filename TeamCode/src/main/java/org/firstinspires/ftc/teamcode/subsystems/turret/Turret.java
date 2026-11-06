package org.firstinspires.ftc.teamcode.subsystems.turret;

import static org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants.*;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.HiveCells;
import org.firstinspires.ftc.teamcode.util.PosePersistency;
import org.firstinspires.ftc.teamcode.vision.CellObservation;
import org.firstinspires.ftc.teamcode.vision.Limelight;

import java.util.function.DoubleUnaryOperator;
import java.util.function.Supplier;

public class Turret extends SubsystemBase {
    public enum State {
        STALL, MANUAL, AIM, IDLE
    }

    private final Servo servo;
    private final DcMotorEx encoder;
    private final Limelight vision;
    private final TurretHistory history;
    private final Supplier<Pose> poseSupplier;
    private final PIDController controller = new PIDController(kP, kI, kD);
    private final InterpLUT timeOfFlightTable = new InterpLUT();

    private Pose botPose = new Pose(0, 0, 0);
    private Pose virtualPose = new Pose(0, 0, 0);
    private Field.Alliance alliance = null;
    private HiveCells.Cell cell = null;

    private State state = State.IDLE;
    private State requestedState = State.IDLE;
    private boolean recovering = false;
    private double stallStart = Double.NaN;
    private double stallTime = Double.NaN;
    private double manualAngle = 0;
    private double zeroTicks = 0;
    private double ticks = 0;
    private boolean encoderFault = false;
    private double angle = 0;
    private double velocity = 0;
    private double power = 0;
    private double previousTime = Double.NaN;
    private double previousAngle = 0;

    private double odomAngle = 0;
    private double bias = 0;
    private double target = 0;
    private double setpoint = 0;
    private boolean inRange = true;
    private double distance = Double.NaN;
    private double visionDistance = Double.NaN;
    private double lastSeenTime = Double.NaN;
    private boolean seen = false;

    private double poseTime = Double.NaN;
    private double lastX = 0;
    private double lastY = 0;
    private double velocityX = 0;
    private double velocityY = 0;
    private double leadTime = 0;

    public Turret(HardwareMap hardwareMap, TurretHistory history) {
        this(hardwareMap, null, history, () -> new Pose(0, 0, 0));
    }

    public Turret(HardwareMap hardwareMap, Limelight vision, TurretHistory history, Supplier<Pose> poseSupplier) {
        this.vision = vision;
        this.poseSupplier = poseSupplier;
        this.history = history;
        servo = hardwareMap.get(Servo.class, SERVO_NAME);
        encoder = hardwareMap.get(DcMotorEx.class, ENCODER_NAME);
        zeroTicks = PosePersistency.getTurretZero(encoder.getCurrentPosition());
        PosePersistency.saveTurretZero(zeroTicks);
        for (int i = 0; i < Math.min(TOF_DIST_IN.length, TOF_S.length); i++) {
            timeOfFlightTable.add(TOF_DIST_IN[i], TOF_S[i]);
        }
        timeOfFlightTable.createLUT();
    }

    public void setAlliance(Field.Alliance alliance) {
        if (alliance != this.alliance) {
            bias = 0;
            lastSeenTime = Double.NaN;
        }
        this.alliance = alliance;
    }

    public void setState(State state) {
        requestedState = state;
    }

    public State getRequestedState() {
        return requestedState;
    }

    private State autoState() {
        if (requestedState == State.AIM && !hasReachableTarget()) return State.IDLE;
        return requestedState;
    }

    private boolean hasReachableTarget() {
        return cell != null && target >= Math.toRadians(MIN_ANGLE_DEG) && target <= Math.toRadians(MAX_ANGLE_DEG);
    }

    private void changeState(State state) {
        if (state == this.state) return;
        if (state == State.STALL) stallTime = history.now();
        controller.reset();
        stallStart = Double.NaN;
        this.state = state;
    }

    public State getState() {
        return state;
    }

    public void setManualAngle(double radians) {
        manualAngle = radians;
        setState(State.MANUAL);
    }

    public void zeroHere() {
        zeroTicks = encoder.getCurrentPosition();
        PosePersistency.saveTurretZero(zeroTicks);
    }

    public void readEncoder() {
        ticks = encoder.getCurrentPosition() - zeroTicks;
        angle = ticksToAngle(ticks, TICKS_PER_REV, ENCODER_GEAR_RATIO, ENCODER_REVERSED);
        encoderFault = angle < Math.toRadians(MIN_ANGLE_DEG - SANITY_MARGIN_DEG)
                || angle > Math.toRadians(MAX_ANGLE_DEG + SANITY_MARGIN_DEG);
    }

    @Override
    public void periodic() {
        double now = history.now();
        botPose = poseSupplier.get();
        updateVelocity(now);
        updateAim(now);
        updateTurret(now);
    }

    private void updateVelocity(double now) {
        if (!Double.isNaN(poseTime) && now > poseTime) {
            double vx = (botPose.x() - lastX) / (now - poseTime);
            double vy = (botPose.y() - lastY) / (now - poseTime);
            if (Math.hypot(vx, vy) <= MAX_ROBOT_SPEED_IN_S) {
                velocityX += POSE_VELOCITY_ALPHA * (vx - velocityX);
                velocityY += POSE_VELOCITY_ALPHA * (vy - velocityY);
            }
        }
        poseTime = now;
        lastX = botPose.x();
        lastY = botPose.y();
    }

    private double timeOfFlight(double inches) {
        return timeOfFlightTable.get(inches);
    }

    private void updateAim(double now) {
        double dt = Double.isNaN(previousTime) ? 0 : now - previousTime;
        HiveCells.Cell nextCell = alliance == null ? null : HiveCells.forRobot(alliance, botPose);
        if (nextCell != cell) {
            cell = nextCell;
            bias = 0;
            lastSeenTime = Double.NaN;
        }
        if (cell == null) {

            target = 0;
            distance = Double.NaN;
            seen = false;
            leadTime = 0;
            virtualPose = botPose;
            return;
        }

        double[] pivot = pivot(botPose.x(), botPose.y(), botPose.heading(), TURRET_FWD, TURRET_LEFT);
        odomAngle = odomAngle(pivot[0], pivot[1], botPose.heading(), cell.pose.x(), cell.pose.y());
        double odomDistance = Math.hypot(cell.pose.x() - pivot[0], cell.pose.y() - pivot[1]);
        double vx = SHOOT_ON_THE_MOVE ? velocityX : 0;
        double vy = SHOOT_ON_THE_MOVE ? velocityY : 0;
        double[] leadPivot = leadPivot(pivot[0], pivot[1], vx, vy, cell.pose.x(), cell.pose.y(),
                TRANSFER_DELAY_S, this::timeOfFlight);
        leadTime = leadPivot[2];
        virtualPose = new Pose(botPose.x() + vx * leadTime, botPose.y() + vy * leadTime, botPose.heading());
        double leadAngle = odomAngle(leadPivot[0], leadPivot[1], botPose.heading(), cell.pose.x(), cell.pose.y());
        double leadDistance = Math.hypot(cell.pose.x() - leadPivot[0], cell.pose.y() - leadPivot[1]);

        CellObservation obs = USE_VISION && vision != null && vision.hasNewFrame() ? vision.getObservation(cell) : null;
        if (obs != null) {
            double visionAngle = visionAngle(obs.turretAngleAtCapture, obs.bearing, botPose.heading(), obs.headingAtCapture);
            double error = wrap(visionAngle - odomAngle);
            if (Math.abs(error) <= Math.toRadians(VISION_MAX_DISAGREE_DEG)) {
                bias += VISION_BIAS_ALPHA * (error - bias);
                visionDistance = obs.distance;
                lastSeenTime = now;
            }
        }

        seen = !Double.isNaN(lastSeenTime) && (now - lastSeenTime) * 1000 <= VISION_LOCK_VALID_MS;
        if (!seen) bias *= Math.exp(-BIAS_DECAY_PER_S * dt);
        target = wrap(leadAngle + bias + Math.toRadians(AIM_OFFSET_DEG));
        distance = (seen ? visionDistance : odomDistance) + leadDistance - odomDistance;
    }

    private void updateTurret(double now) {
        servo.setDirection(SERVO_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);

        readEncoder();
        if (encoderFault) {
            setPower(0);
            previousTime = Double.NaN;
            return;
        }

        if (!Double.isNaN(previousTime)) {
            double dt = now - previousTime;
            if (dt > 0) velocity += VELOCITY_FILTER_ALPHA * ((angle - previousAngle) / dt - velocity);
        }
        previousTime = now;
        previousAngle = angle;
        history.add(now, angle, botPose.x(), botPose.y(), botPose.heading());

        if (state == State.STALL) {
            controller.reset();
            setpoint = angle;
            setPower(0);
            if (requestedState != State.STALL && now - stallTime >= STALL_COOLDOWN_S) {
                recovering = true;
                changeState(State.IDLE);
            }
            return;
        }
        if (!recovering) changeState(autoState());
        if (state == State.STALL) {
            setPower(0);
            return;
        }

        double min = Math.toRadians(MIN_ANGLE_DEG);
        double max = Math.toRadians(MAX_ANGLE_DEG);
        double goal;
        switch (state) {
            case AIM:
                goal = target;
                break;
            case MANUAL:
                goal = manualAngle;
                break;
            default:
                goal = Math.toRadians(IDLE_ANGLE_DEG);
        }
        inRange = goal >= min && goal <= max;
        setpoint = inRange ? goal : 0;

        controller.setPID(kP, kI, kD);
        double error = setpoint - angle;
        double output = controller.calculate(angle, setpoint);
        if (Math.abs(error) > Math.toRadians(DEADBAND_DEG)) output += kS * Math.signum(error);
        output = Range.clip(output, -0.8, 0.8);

        if ((angle >= max && output > 0) || (angle <= min && output < 0)) output = 0;

        if (!isPushingWithoutMoving(output, error, velocity)) {
            stallStart = Double.NaN;
        } else if (Double.isNaN(stallStart)) {
            stallStart = now;
        } else if (now - stallStart >= STALL_TIME_S) {
            changeState(State.STALL);
            setPower(0);
            return;
        }
        setPower(output);

        if (recovering && state == State.IDLE && isSettled()) recovering = false;
    }

    static boolean isPushingWithoutMoving(double output, double error, double velocity) {
        return Math.abs(output) >= STALL_MIN_POWER
                && Math.abs(error) > Math.toRadians(AIM_TOLERANCE_DEG)
                && Math.abs(velocity) < Math.toRadians(STALL_VEL_DEG_S);
    }

    private void setPower(double output) {
        power = output;
        servo.setPosition(0.5 + 0.5 * output);
    }

    public boolean isTooClose() {
        return seen && visionDistance < MIN_SHOT_DIST;
    }

    public boolean isSettled() {
        return !encoderFault
                && Math.abs(setpoint - angle) < Math.toRadians(AIM_TOLERANCE_DEG)
                && Math.abs(velocity) < Math.toRadians(SETTLED_VEL_DEG_S);
    }

    public boolean isTooFast() {
        return getRobotSpeed() > MAX_SHOOT_SPEED_IN_S;
    }

    public double getRobotSpeed() {
        return Math.hypot(velocityX, velocityY);
    }

    public boolean isAimed() {
        return state == State.AIM && cell != null && inRange && isSettled()
                && !isTooClose() && distance <= MAX_SHOT_DIST && !isTooFast()
                && (seen || ALLOW_ODOMETRY_ONLY_SHOTS);
    }

    public void addTelemetry(TelemetryData telemetry) {
        telemetry.addData("Turret state", recovering ? state + " (recovering)" : state);
        telemetry.addData("Turret requested state", requestedState);
        telemetry.addData("Turret aimed", isAimed());
        telemetry.addData("Robot speed in/s", getRobotSpeed());
        telemetry.addData("Turret too fast", isTooFast());
        telemetry.addData("Lead time s", leadTime);
        telemetry.addData("Turret cell", cell == null ? "none" : cell);
        telemetry.addData("Turret tags seen", seen);
        telemetry.addData("Turret too close", isTooClose());
        telemetry.addData("Turret in range", inRange);
        telemetry.addData("Turret settled", isSettled());
        telemetry.addData("Turret distance in", distance);
        telemetry.addData("Turret odom deg", Math.toDegrees(odomAngle));
        telemetry.addData("Turret bias deg", Math.toDegrees(bias));
        telemetry.addData("Turret aim offset deg", AIM_OFFSET_DEG);
        telemetry.addData("Turret target deg", Math.toDegrees(target));
        telemetry.addData("Turret angle deg", Math.toDegrees(angle));
        telemetry.addData("Turret encoder ticks", ticks);
        telemetry.addData("Turret encoder fault", encoderFault);
    }

    static double ticksToAngle(double ticks, double ticksPerRev, double encoderGearRatio, boolean reversed) {
        double radians = ticks / ticksPerRev / encoderGearRatio * 2 * Math.PI;
        return reversed ? -radians : radians;
    }

    public static double[] pivot(double x, double y, double heading, double pivotForward, double pivotLeft) {
        double c = Math.cos(heading), s = Math.sin(heading);
        return new double[]{x + pivotForward * c - pivotLeft * s, y + pivotForward * s + pivotLeft * c};
    }

    public static double odomAngle(double pivotX, double pivotY, double heading, double aimX, double aimY) {
        return wrap(Math.atan2(aimY - pivotY, aimX - pivotX) - heading);
    }

    public static double visionAngle(double turretAtCapture, double bearing, double headingNow, double headingAtCapture) {
        return wrap(turretAtCapture + bearing - wrap(headingNow - headingAtCapture));
    }

    static double[] leadPivot(double x, double y, double velX, double velY, double aimX, double aimY,
                              double transferDelay, DoubleUnaryOperator timeOfFlight) {
        double t = transferDelay + timeOfFlight.applyAsDouble(Math.hypot(aimX - x, aimY - y));
        for (int i = 0; i < 3; i++) {
            t = transferDelay + timeOfFlight.applyAsDouble(Math.hypot(aimX - (x + velX * t), aimY - (y + velY * t)));
        }
        return new double[]{x + velX * t, y + velY * t, t};
    }

    public static double wrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    public double getAngle() {
        return angle;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getPower() {
        return power;
    }

    public double getTicks() {
        return ticks;
    }

    public boolean isEncoderFault() {
        return encoderFault;
    }

    public boolean isInRange() {
        return inRange;
    }

    public double getBias() {
        return bias;
    }

    public double getTarget() {
        return target;
    }

    public boolean isSeen() {
        return seen;
    }

    public double getDistance() {
        return distance;
    }

    public Pose getVirtualPose() {
        return virtualPose;
    }

    public HiveCells.Cell getCell() {
        return cell;
    }
}
