package org.firstinspires.ftc.teamcode.subsystems.turretservo;

import static org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants.*;
import static org.firstinspires.ftc.teamcode.subsystems.turretservo.TurretServoConstants.*;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.HiveCells;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretHistory;
import org.firstinspires.ftc.teamcode.vision.CellObservation;
import org.firstinspires.ftc.teamcode.vision.Limelight;

public class TurretServo extends SubsystemBase {
    private final ServoImplEx servo;
    private final DcMotorEx shooter;
    private final Limelight vision;
    private final TurretHistory history;
    private final InterpLUT shooterTable = new InterpLUT();

    private Pose botPose = new Pose(0, 0, 0);
    private Field.Alliance alliance = null;
    private HiveCells.Cell cell = null;

    private double manualAngle = Double.NaN;
    private double angle = 0;
    private double position = Double.NaN;
    private double previousTime = Double.NaN;

    private double odomAngle = 0;
    private double bias = 0;
    private double target = 0;
    private double setpoint = 0;
    private boolean inRange = true;
    private double distance = Double.NaN;
    private double visionDistance = Double.NaN;
    private double lastSeenTime = Double.NaN;
    private boolean seen = false;

    private boolean shooterEnabled = false;
    private double shooterTarget = 0;

    public TurretServo(HardwareMap hardwareMap, TurretHistory history) {
        this(hardwareMap, null, history);
    }

    public TurretServo(HardwareMap hardwareMap, Limelight vision, TurretHistory history) {
        this.vision = vision;
        this.history = history;
        servo = hardwareMap.get(ServoImplEx.class, TurretServoConstants.SERVO_NAME);
        shooter = hardwareMap.get(DcMotorEx.class, ShooterConstants.MOTOR_NAME);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        for (int i = 0; i < Math.min(ShooterConstants.SHOOTER_DIST_IN.length, ShooterConstants.SHOOTER_RPM.length); i++) {
            shooterTable.add(ShooterConstants.SHOOTER_DIST_IN[i], ShooterConstants.SHOOTER_RPM[i]);
        }
        shooterTable.createLUT();
    }

    public void updateBotPose(Pose pose) {
        botPose = pose;
    }

    public void setAlliance(Field.Alliance alliance) {
        if (alliance != this.alliance) {
            bias = 0;
            lastSeenTime = Double.NaN;
        }
        this.alliance = alliance;
    }

    public void setManualAngle(double radians) {
        manualAngle = radians;
    }

    public void clearManualAngle() {
        manualAngle = Double.NaN;
    }

    public void setShooterEnabled(boolean enabled) {
        shooterEnabled = enabled;
    }

    @Override
    public void periodic() {
        double now = history.now();
        updateAim(now);
        updateTurret(now);
        updateShooter();
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
            return;
        }

        double[] pivot = Turret.pivot(botPose.x(), botPose.y(), botPose.heading(), TURRET_FWD, TURRET_LEFT);
        odomAngle = Turret.odomAngle(pivot[0], pivot[1], botPose.heading(), cell.pose.x(), cell.pose.y());
        double odomDistance = Math.hypot(cell.pose.x() - pivot[0], cell.pose.y() - pivot[1]);

        CellObservation obs = vision != null && vision.hasNewFrame() ? vision.getObservation(cell) : null;
        if (obs != null) {
            double visionAngle = Turret.visionAngle(obs.turretAngleAtCapture, obs.bearing, botPose.heading(), obs.headingAtCapture);
            double error = Turret.wrap(visionAngle - odomAngle);
            if (Math.abs(error) <= Math.toRadians(VISION_MAX_DISAGREE_DEG)) {
                bias = error;
                visionDistance = obs.distance;
                lastSeenTime = now;
            }
        }

        seen = !Double.isNaN(lastSeenTime) && (now - lastSeenTime) * 1000 <= VISION_LOCK_VALID_MS;
        if (!seen) bias *= Math.exp(-BIAS_DECAY_PER_S * dt);
        target = Turret.wrap(odomAngle + bias);
        distance = seen ? visionDistance : odomDistance;
    }

    private void updateTurret(double now) {
        double dt = Double.isNaN(previousTime) ? 0 : now - previousTime;
        previousTime = now;

        double min = Math.toRadians(MIN_ANGLE_DEG);
        double max = Math.toRadians(MAX_ANGLE_DEG);
        double goal = Double.isNaN(manualAngle) ? target : manualAngle;
        inRange = goal >= min && goal <= max;
        setpoint = inRange ? goal : 0;

        position = angleToPosition(setpoint, CENTER_POSITION, GEAR_RATIO, SERVO_RANGE_DEG, TurretServoConstants.SERVO_REVERSED);
        servo.setPosition(position);

        angle = estimateAngle(angle, setpoint, Math.toRadians(TURRET_SPEED_DEG_S) * dt);
        history.add(now, angle, botPose.x(), botPose.y(), botPose.heading());
    }

    private void updateShooter() {
        shooter.setDirection(ShooterConstants.MOTOR_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        shooterTarget = Double.isNaN(distance) ? 0 : shooterTable.get(distance);
        if (shooterEnabled && shooterTarget > 0) {
            shooter.setVelocity(shooterTarget * ShooterConstants.TICKS_PER_REV / 60.0);
        } else {
            shooter.setPower(0);
        }
    }

    public boolean isTooClose() {
        return seen && visionDistance < MIN_SHOT_DIST;
    }

    public boolean isSettled() {
        return Math.abs(setpoint - angle) < Math.toRadians(AIM_TOLERANCE_DEG);
    }

    public boolean isShooterReady() {
        return shooterEnabled && shooterTarget > 0
                && Math.abs(shooter.getVelocity() * 60.0 / ShooterConstants.TICKS_PER_REV - shooterTarget) <= ShooterConstants.SHOOTER_TOLERANCE_RPM;
    }

    public boolean okToShoot() {
        return cell != null && Double.isNaN(manualAngle) && inRange && isSettled()
                && isShooterReady() && !isTooClose() && distance <= MAX_SHOT_DIST
                && (seen || ALLOW_ODOMETRY_ONLY_SHOTS);
    }

    public void addTelemetry(TelemetryData telemetry) {
        telemetry.addData("Turret OK to shoot", okToShoot());
        telemetry.addData("Turret cell", cell == null ? "none" : cell);
        telemetry.addData("Turret tags seen", seen);
        telemetry.addData("Turret too close", isTooClose());
        telemetry.addData("Turret in range", inRange);
        telemetry.addData("Turret settled", isSettled());
        telemetry.addData("Turret distance in", distance);
        telemetry.addData("Turret odom deg", Math.toDegrees(odomAngle));
        telemetry.addData("Turret bias deg", Math.toDegrees(bias));
        telemetry.addData("Turret target deg", Math.toDegrees(target));
        telemetry.addData("Turret setpoint deg", Math.toDegrees(setpoint));
        telemetry.addData("Turret estimated deg", Math.toDegrees(angle));
        telemetry.addData("Turret servo position", position);
        telemetry.addData("Shooter target", shooterTarget);
        telemetry.addData("Shooter velocity", shooter.getVelocity());
    }

    static double angleToPosition(double radians, double centerPosition, double gearRatio, double servoRangeDeg, boolean reversed) {
        double servoDeg = Math.toDegrees(radians) * gearRatio;
        return Range.clip(centerPosition + (reversed ? -servoDeg : servoDeg) / servoRangeDeg, 0, 1);
    }

    static double estimateAngle(double angle, double setpoint, double maxStep) {
        return angle + Range.clip(setpoint - angle, -maxStep, maxStep);
    }

    public double getAngle() {
        return angle;
    }

    public double getPosition() {
        return position;
    }

    public double getDistance() {
        return distance;
    }

    public HiveCells.Cell getCell() {
        return cell;
    }
}
