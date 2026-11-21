package org.firstinspires.ftc.teamcode.subsystems.turretmotor;

import static org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants.*;
import static org.firstinspires.ftc.teamcode.subsystems.turretmotor.TurretMotorConstants.*;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.HiveCells;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretHistory;
import org.firstinspires.ftc.teamcode.util.PosePersistency;
import org.firstinspires.ftc.teamcode.vision.CellObservation;
import org.firstinspires.ftc.teamcode.vision.Limelight;

import java.util.function.Supplier;

public class TurretMotor extends SubsystemBase {
    public enum State {
        STALL, MANUAL, AIM, IDLE
    }

    private final DcMotorEx motor;
    private final Limelight vision;
    private final TurretHistory history;
    private final Supplier<Pose> poseSupplier;
    private final PIDController controller = new PIDController(MOTOR_kP, MOTOR_kI, MOTOR_kD);
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
    private double previousAimTime = Double.NaN;
    private double previousControlTime = Double.NaN;
    private double previousTarget = Double.NaN;
    private double targetRate = 0;
    private double integral = 0;
    private double feedforward = 0;

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

    public TurretMotor(HardwareMap hardwareMap, TurretHistory history) {
        this(hardwareMap, null, history, () -> new Pose(0, 0, 0));
    }

    public TurretMotor(HardwareMap hardwareMap, Limelight vision, TurretHistory history, Supplier<Pose> poseSupplier) {
        this.vision = vision;
        this.poseSupplier = poseSupplier;
        this.history = history;
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        motor.setDirection(MOTOR_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        zeroTicks = PosePersistency.getTurretMotorZero(motor.getCurrentPosition());
        PosePersistency.saveTurretMotorZero(zeroTicks);
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
        return cell != null && target >= Math.toRadians(MOTOR_MIN_ANGLE_DEG) && target <= Math.toRadians(MOTOR_MAX_ANGLE_DEG);
    }

    private void changeState(State state) {
        if (state == this.state) return;
        if (state == State.STALL) stallTime = history.now();
        controller.reset();
        integral = 0;
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
        zeroTicks = motor.getCurrentPosition();
        PosePersistency.saveTurretMotorZero(zeroTicks);
    }

    public void readEncoder() {
        ticks = motor.getCurrentPosition() - zeroTicks;
        angle = ticksToAngle(ticks);
        encoderFault = angle < Math.toRadians(MOTOR_MIN_ANGLE_DEG - MOTOR_SANITY_MARGIN_DEG)
                || angle > Math.toRadians(MOTOR_MAX_ANGLE_DEG + MOTOR_SANITY_MARGIN_DEG);
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
        double dt = Double.isNaN(previousAimTime) ? 0 : now - previousAimTime;
        previousAimTime = now;
        HiveCells.Cell nextCell = alliance == null ? null : HiveCells.forRobot(alliance, botPose);
        if (nextCell != cell) {
            cell = nextCell;
            bias = 0;
            lastSeenTime = Double.NaN;
            previousTarget = Double.NaN;
            targetRate = 0;
        }
        if (cell == null) {
            previousTarget = Double.NaN;
            targetRate = 0;
            target = 0;
            distance = Double.NaN;
            seen = false;
            leadTime = 0;
            virtualPose = botPose;
            return;
        }

        double[] pivot = Turret.pivot(botPose.x(), botPose.y(), botPose.heading(), MOTOR_TURRET_FWD, MOTOR_TURRET_LEFT);
        odomAngle = Turret.odomAngle(pivot[0], pivot[1], botPose.heading(), cell.pose.x(), cell.pose.y());
        double odomDistance = Math.hypot(cell.pose.x() - pivot[0], cell.pose.y() - pivot[1]);
        double vx = SHOOT_ON_THE_MOVE ? velocityX : 0;
        double vy = SHOOT_ON_THE_MOVE ? velocityY : 0;
        double[] leadPivot = Turret.leadPivot(pivot[0], pivot[1], vx, vy, cell.pose.x(), cell.pose.y(),
                TRANSFER_DELAY_S, this::timeOfFlight);
        leadTime = leadPivot[2];
        virtualPose = new Pose(botPose.x() + vx * leadTime, botPose.y() + vy * leadTime, botPose.heading());
        double leadAngle = Turret.odomAngle(leadPivot[0], leadPivot[1], botPose.heading(), cell.pose.x(), cell.pose.y());
        double leadDistance = Math.hypot(cell.pose.x() - leadPivot[0], cell.pose.y() - leadPivot[1]);

        CellObservation obs = USE_VISION && vision != null && vision.hasNewFrame() ? vision.getObservation(cell) : null;
        if (obs != null) {
            double visionAngle = Turret.visionAngle(obs.turretAngleAtCapture, obs.bearing, botPose.heading(), obs.headingAtCapture);
            double error = Turret.wrap(visionAngle - odomAngle);
            if (Math.abs(error) <= Math.toRadians(VISION_MAX_DISAGREE_DEG)) {
                bias += VISION_BIAS_ALPHA * (error - bias);
                visionDistance = obs.distance;
                lastSeenTime = now;
            }
        }

        seen = !Double.isNaN(lastSeenTime) && (now - lastSeenTime) * 1000 <= VISION_LOCK_VALID_MS;
        if (!seen) bias *= Math.exp(-BIAS_DECAY_PER_S * dt);
        target = Turret.wrap(leadAngle + bias + Math.toRadians(AIM_OFFSET_DEG));
        if (!Double.isNaN(previousTarget) && dt > 0) {
            double maxRate = Math.toRadians(MOTOR_FF_MAX_RATE_DEG_S);
            double rate = Range.clip(Turret.wrap(target - previousTarget) / dt, -maxRate, maxRate);
            targetRate += MOTOR_FF_FILTER_ALPHA * (rate - targetRate);
        }
        previousTarget = target;
        distance = (seen ? visionDistance : odomDistance) + leadDistance - odomDistance;
    }

    private void updateTurret(double now) {
        double dt = Double.isNaN(previousControlTime) ? 0 : now - previousControlTime;
        previousControlTime = now;
        readEncoder();
        if (encoderFault) {
            setPower(0);
            return;
        }

        double measured = ticksToAngle(motor.getVelocity());
        velocity += MOTOR_VELOCITY_FILTER_ALPHA * (measured - velocity);
        history.add(now, angle, botPose.x(), botPose.y(), botPose.heading());

        if (state == State.STALL) {
            controller.reset();
            setpoint = angle;
            setPower(0);
            if (requestedState != State.STALL && now - stallTime >= MOTOR_STALL_COOLDOWN_S) {
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

        double min = Math.toRadians(MOTOR_MIN_ANGLE_DEG);
        double max = Math.toRadians(MOTOR_MAX_ANGLE_DEG);
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

        controller.setPID(MOTOR_kP, 0, MOTOR_kD);
        double error = setpoint - angle;
        double output = controller.calculate(angle, setpoint);

        if (Math.abs(error) > Math.toRadians(MOTOR_I_ZONE_DEG)) {
            integral = 0;
        } else if (Math.abs(error) > Math.toRadians(MOTOR_DEADBAND_DEG) && MOTOR_kI > 0) {
            double limit = MOTOR_I_MAX_POWER / MOTOR_kI;
            integral = Range.clip(integral + error * dt, -limit, limit);
        }
        output += MOTOR_kI * integral;

        feedforward = state == State.AIM && inRange ? MOTOR_kV * targetRate : 0;
        output += feedforward;

        if (Math.abs(error) > Math.toRadians(MOTOR_DEADBAND_DEG)) output += MOTOR_kS * Math.signum(error);
        output = Range.clip(output, -MOTOR_MAX_POWER, MOTOR_MAX_POWER);

        if ((angle >= max && output > 0) || (angle <= min && output < 0)) output = 0;

        if (!isPushingWithoutMoving(output, error, velocity)) {
            stallStart = Double.NaN;
        } else if (Double.isNaN(stallStart)) {
            stallStart = now;
        } else if (now - stallStart >= MOTOR_STALL_TIME_S) {
            changeState(State.STALL);
            setPower(0);
            return;
        }
        setPower(output);

        if (recovering && state == State.IDLE && isSettled()) recovering = false;
    }

    static boolean isPushingWithoutMoving(double output, double error, double velocity) {
        return Math.abs(output) >= MOTOR_STALL_MIN_POWER
                && Math.abs(error) > Math.toRadians(MOTOR_AIM_TOLERANCE_DEG)
                && Math.abs(velocity) < Math.toRadians(MOTOR_STALL_VEL_DEG_S);
    }

    public static double ticksToAngle(double ticks) {
        return ticks / MOTOR_TICKS_PER_REV / MOTOR_GEAR_RATIO * 2 * Math.PI;
    }

    private void setPower(double output) {
        power = output;
        motor.setPower(output);
    }

    public boolean isTooClose() {
        return seen && visionDistance < MIN_SHOT_DIST;
    }

    public boolean isSettled() {
        return !encoderFault
                && Math.abs(setpoint - angle) < Math.toRadians(MOTOR_AIM_TOLERANCE_DEG)
                && Math.abs(velocity) < Math.toRadians(MOTOR_SETTLED_VEL_DEG_S);
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
        telemetry.addData("Lead time s", leadTime);
        telemetry.addData("Turret cell", cell == null ? "none" : cell);
        telemetry.addData("Turret tags seen", seen);
        telemetry.addData("Turret in range", inRange);
        telemetry.addData("Turret settled", isSettled());
        telemetry.addData("Turret distance in", distance);
        telemetry.addData("Turret aim offset deg", AIM_OFFSET_DEG);
        telemetry.addData("Turret target deg", Math.toDegrees(target));
        telemetry.addData("Turret angle deg", Math.toDegrees(angle));
        telemetry.addData("Turret velocity deg/s", Math.toDegrees(velocity));
        telemetry.addData("Turret error deg", Math.toDegrees(setpoint - angle));
        telemetry.addData("Turret target rate deg/s", Math.toDegrees(targetRate));
        telemetry.addData("Turret feedforward", feedforward);
        telemetry.addData("Turret integral power", MOTOR_kI * integral);
        telemetry.addData("Turret power", power);
        telemetry.addData("Turret encoder ticks", ticks);
        telemetry.addData("Turret encoder fault", encoderFault);
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
