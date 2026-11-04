package org.firstinspires.ftc.teamcode.kiwi;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.revhub.drivetrains.CachedMotor;
import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.HashMap;
import java.util.Map;

/**
 * 3-wheel Kiwi drivetrain for Pedro Pathing 3, structured after the revhub Mecanum drivetrain.
 * +forward is +X, +strafe is left (+Y) and +turn is CCW (top-down), matching Pedro's convention.
 * Wheels are spaced 120 degrees apart: Motor A drives along the Y axis (left/right),
 * Motor B is front-left and Motor C is front-right.
 */
public class Kiwi implements Drivetrain {
    public final KiwiConfig config;
    private final CachedMotor[] motors;
    private final VoltageSensor voltageSensor;
    public final double[] wheelPowers = new double[3];

    private static final int A = 0;
    private static final int B = 1;
    private static final int C = 2;

    private static final double SIN_60 = Math.sqrt(3) / 2.0;
    /** The battery voltage is read at most this often, as each read takes loop time. */
    private static final double VOLTAGE_READ_INTERVAL_SECONDS = 0.1;

    private double powerScale = 1.0;
    private DrivePowers drivePowers = DrivePowers.zero();
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private double voltage = Double.NaN;

    public Kiwi(HardwareMap map, KiwiConfig config) {
        this.config = config;
        double powerDeadband = config.powerThreshold.get();

        motors = new CachedMotor[]{
                new CachedMotor(map.get(DcMotorEx.class, config.motorAName.get()), powerDeadband),
                new CachedMotor(map.get(DcMotorEx.class, config.motorBName.get()), powerDeadband),
                new CachedMotor(map.get(DcMotorEx.class, config.motorCName.get()), powerDeadband)
        };

        for (CachedMotor motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.raw().getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.raw().setMotorType(motorConfigurationType);
        }

        motors[A].setDirection(config.motorADirection.get());
        motors[B].setDirection(config.motorBDirection.get());
        motors[C].setDirection(config.motorCDirection.get());

        voltageSensor = map.voltageSensor.iterator().next();

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void applyDrive(DrivePowers powers) {
        applyDrive(powers, false);
    }

    /**
     * With turnFirst (driver control), a wheel over full power scales down only the translation, so
     * the turn (e.g. auto-aim's) keeps its full power while the driver sprints. Otherwise all three
     * wheels scale down together, keeping the direction of travel (path following).
     */
    public void applyDrive(DrivePowers powers, boolean turnFirst) {
        drivePowers = powers;
        double voltageNormalized = config.voltageCompensation.get() ? getVoltageNormalized() : 1.0;

        if (turnFirst) {
            double turn = Math.max(-1.0, Math.min(1.0, powers.turn() * voltageNormalized));
            double[] translation = computeWheelPowersUnnormalized(new DrivePowers(powers.forward(), powers.strafe(), 0.0));
            // The largest share of the translation (0..1) that keeps every wheel within full power
            double scale = 1.0;
            for (double t : translation) {
                double wheel = t * voltageNormalized;
                if (Math.abs(wheel) < 1e-9) continue;
                double limit = ((wheel > 0 ? 1.0 : -1.0) - turn) / wheel;
                scale = Math.min(scale, Math.max(0.0, limit));
            }
            powerScale = scale;
            for (int i = 0; i < translation.length; i++) {
                wheelPowers[i] = translation[i] * voltageNormalized * scale + turn;
                motors[i].setPower(wheelPowers[i]);
            }
            return;
        }

        double[] requested = computeWheelPowersUnnormalized(powers);
        for (int i = 0; i < requested.length; i++) {
            requested[i] *= voltageNormalized;
        }

        // Scale all three down together if any is over full power, keeping the direction of travel
        double maxPower = 1.0;
        for (double power : requested) {
            maxPower = Math.max(maxPower, Math.abs(power));
        }
        powerScale = 1.0 / maxPower;

        for (int i = 0; i < requested.length; i++) {
            wheelPowers[i] = requested[i] / maxPower;
            motors[i].setPower(wheelPowers[i]);
        }
    }

    public double[] computeWheelPowersUnnormalized(DrivePowers powers) {
        double[] wheelPowers = new double[3];
        double forward = powers.forward();
        double strafe = powers.strafe();
        double turn = powers.turn();

        wheelPowers[A] = -strafe + turn;
        wheelPowers[B] = -SIN_60 * forward + 0.5 * strafe + turn;
        wheelPowers[C] = SIN_60 * forward + 0.5 * strafe + turn;

        return wheelPowers;
    }

    @Override
    public double maxScaling(DrivePowers current, DrivePowers delta) {
        double lambda = 1.0;

        double[] currentPowers = computeWheelPowersUnnormalized(current);
        double[] deltaPowers = computeWheelPowersUnnormalized(delta);

        for (int i = 0; i < 3; i++) {
            double a = currentPowers[i];
            double b = deltaPowers[i];

            if (Math.abs(b) < 1e-9) continue;

            double t1 = (1.0 - a) / b;
            double t2 = (-1.0 - a) / b;

            if (t1 >= 0.0 && t1 < lambda) lambda = t1;
            if (t2 >= 0.0 && t2 < lambda) lambda = t2;
        }

        return Utils.clamp(lambda, 0.0, 1.0);
    }

    @Override
    public void drive(DrivePowers powers, boolean manual) {
        setZeroPowerBehavior(manual && config.manualBrakeMode.get()
                ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
        applyDrive(powers, manual);
    }

    @Override
    public void stop() {
        stop(config.manualBrakeMode.get());
    }

    @Override
    public void stop(boolean brake) {
        setZeroPowerBehavior(brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);

        for (int i = 0; i < motors.length; i++) {
            wheelPowers[i] = 0;
            motors[i].setPower(0);
        }
    }

    @Override
    public Map<String, Object> debug() {
        Map<String, Object> map = new HashMap<>();

        map.put("forward", drivePowers.forward());
        map.put("strafe", drivePowers.strafe());
        map.put("turn", drivePowers.turn());
        map.put("powerScale", powerScale);
        map.put("motorAPower", wheelPowers[A]);
        map.put("motorBPower", wheelPowers[B]);
        map.put("motorCPower", wheelPowers[C]);

        return map;
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        for (CachedMotor motor : motors) {
            motor.setZeroPowerBehavior(behavior);
        }
    }

    /**
     * Total current of the three motors in amps. Motor current isn't in the bulk read, so this
     * makes three hardware reads: avoid calling it every loop.
     */
    public double currentAmps() {
        double total = 0;
        for (CachedMotor motor : motors) {
            total += motor.raw().getCurrent(CurrentUnit.AMPS);
        }
        return total;
    }

    /** Scale factor for voltage compensation, from a battery reading at most VOLTAGE_READ_INTERVAL_SECONDS old. */
    private double getVoltageNormalized() {
        if (Double.isNaN(voltage) || voltageTimer.seconds() > VOLTAGE_READ_INTERVAL_SECONDS) {
            voltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }
        double nominalVoltage = config.nominalVoltage.get();
        double staticFrictionCoefficient = config.staticFrictionCoefficient.get();
        return (nominalVoltage - (nominalVoltage * staticFrictionCoefficient)) / (voltage
                - ((nominalVoltage * nominalVoltage / voltage) * staticFrictionCoefficient));
    }

    @Override
    public double interpolateVelocity(double xRadius, double yRadius, double theta) {
        return 1.0 / (Math.abs(Math.cos(theta)) / xRadius + Math.abs(Math.sin(theta)) / yRadius);
    }
}
