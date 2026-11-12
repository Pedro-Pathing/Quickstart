package org.firstinspires.ftc.teamcode.subsystems.shooter;

import static org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.util.TelemetryData;

import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {
    public enum State {
        OFF, FIXED, DISTANCE
    }

    private final DcMotorEx flywheel;
    private final DoubleSupplier distance;
    private final PIDFController flywheelController = new PIDFController(FLYWHEEL_PIDF_COEFFICIENTS);
    private final InterpLUT distanceLUT = new InterpLUT();

    private State state = State.OFF;
    private double targetRpm = 0;
    private double flywheelVelocity = 0;

    public Shooter(HardwareMap hardwareMap, DoubleSupplier distance) {
        this.distance = distance;
        flywheel = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        for (int i = 0; i < Math.min(SHOOTER_DIST_IN.length, SHOOTER_RPM.length); i++) {
            distanceLUT.add(SHOOTER_DIST_IN[i], SHOOTER_RPM[i]);
        }
        distanceLUT.createLUT();
    }

    public void setState(State state) {
        if (state != this.state) flywheelController.reset();
        this.state = state;
    }

    public State getState() {
        return state;
    }

    public void setTargetRPM(double rpm) {
        targetRpm = Range.clip(rpm, 0, MAX_RPM);
    }

    public double getTargetRPM() {
        return toRpm(flywheelController.getSetPoint());
    }

    public double getRPM() {
        return toRpm(flywheelVelocity);
    }

    public boolean flywheelReady() {
        return (state == State.FIXED || state == State.DISTANCE) && getTargetRPM() > 0
                && Math.abs(getTargetRPM() - getRPM()) <= SHOOTER_TOLERANCE_RPM;
    }

    @Override
    public void periodic() {
        flywheel.setDirection(MOTOR_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        flywheelController.setCoefficients(FLYWHEEL_PIDF_COEFFICIENTS);
        flywheelVelocity = flywheel.getVelocity();

        switch (state) {
            case OFF:
                flywheelController.reset();
                flywheelController.setSetPoint(0);
                flywheel.setPower(0);
                break;
            case FIXED:
                flywheelController.setSetPoint(toTicks(targetRpm));
                flywheel.setPower(Range.clip(flywheelController.calculate(flywheelVelocity), 0, 1));
                break;
            case DISTANCE:
                double inches = distance.getAsDouble();
                double rpm = Double.isNaN(inches) ? targetRpm : distanceLUT.get(inches);
                flywheelController.setSetPoint(toTicks(rpm));
                flywheel.setPower(Range.clip(flywheelController.calculate(flywheelVelocity), 0, 1));
                break;
        }
    }

    public void addTelemetry(TelemetryData telemetry) {
        telemetry.addData("Shooter state", state);
        telemetry.addData("Shooter target", getTargetRPM());
        telemetry.addData("Current RPM", getRPM());
        telemetry.addData("Shooter ready", flywheelReady());
    }

    private static double toTicks(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }

    private static double toRpm(double ticksPerSecond) {
        return ticksPerSecond * 60.0 / TICKS_PER_REV;
    }
}
