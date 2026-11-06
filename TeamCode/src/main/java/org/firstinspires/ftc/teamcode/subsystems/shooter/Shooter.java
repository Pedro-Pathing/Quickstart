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
    private final DcMotorEx flywheel;
    private final DoubleSupplier distance;
    private final PIDFController flywheelController = new PIDFController(FLYWHEEL_PIDF_COEFFICIENTS);
    private final InterpLUT distanceLUT = new InterpLUT();

    private boolean activeControl = false;
    private boolean distanceControl = true;
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

    public void setFlywheel(double rpm, boolean setActiveControl) {
        flywheelController.setSetPoint(toTicks(Range.clip(rpm, 0, MAX_RPM)));
        setActiveControl(setActiveControl);
    }

    public void setActiveControl(boolean state) {
        activeControl = state;
    }

    public boolean getActiveControl() {
        return activeControl;
    }

    public void setDistanceControl(boolean state) {
        distanceControl = state;
    }

    public boolean getDistanceControl() {
        return distanceControl;
    }

    public double getTargetRPM() {
        return toRpm(flywheelController.getSetPoint());
    }

    public double getRPM() {
        return toRpm(flywheelVelocity);
    }

    public boolean flywheelReady() {
        return activeControl && getTargetRPM() > 0 && Math.abs(getTargetRPM() - getRPM()) <= SHOOTER_TOLERANCE_RPM;
    }

    @Override
    public void periodic() {
        flywheel.setDirection(MOTOR_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        flywheelController.setCoefficients(FLYWHEEL_PIDF_COEFFICIENTS);
        flywheelVelocity = flywheel.getVelocity();

        if (activeControl && distanceControl) {
            double inches = distance.getAsDouble();
            if (!Double.isNaN(inches)) {
                flywheelController.setSetPoint(toTicks(distanceLUT.get(inches)));
            }
        }

        if (activeControl) {
            flywheel.setPower(Range.clip(flywheelController.calculate(flywheelVelocity), 0, 1));
        } else {
            flywheelController.reset();
            flywheel.setPower(flywheelController.getSetPoint() == 0 ? 0 : IDLE_POWER);
        }
    }

    public void addTelemetry(TelemetryData telemetry) {
        telemetry.addData("Shooter", activeControl ? (distanceControl ? "ACTIVE (distance)" : "ACTIVE (fixed RPM)")
                : getTargetRPM() == 0 ? "OFF" : "IDLE");
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
