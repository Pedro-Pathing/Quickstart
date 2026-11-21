package org.firstinspires.ftc.teamcode.subsystems.indicator;

import static org.firstinspires.ftc.teamcode.subsystems.indicator.IndicatorConstants.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.TelemetryData;

import java.util.function.BooleanSupplier;

public class Indicator extends SubsystemBase {
    private final Servo turretLight;
    private final Servo pollenLight;
    private final BooleanSupplier turretAimed;
    private final BooleanSupplier pollenPresent;
    private final ElapsedTime blinkTimer = new ElapsedTime();
    private final ElapsedTime lastSeenTimer = new ElapsedTime();

    private Color turretColor = null;
    private Color pollenColor = null;
    private boolean blinking = false;

    public Indicator(HardwareMap hardwareMap, BooleanSupplier turretAimed) {
        this(hardwareMap, turretAimed, null);
    }

    public Indicator(HardwareMap hardwareMap, BooleanSupplier turretAimed, BooleanSupplier pollenPresent) {
        this.turretAimed = turretAimed;
        this.pollenPresent = pollenPresent;
        turretLight = turretAimed == null ? null : hardwareMap.get(Servo.class, TURRET_LIGHT_NAME);
        pollenLight = pollenPresent == null ? null : hardwareMap.get(Servo.class, POLLEN_LIGHT_NAME);
        setTurretColor(Color.OFF);
        setPollenColor(Color.OFF);
    }

    public void setTurretColor(Color color) {
        if (turretLight == null || color == turretColor) return;
        turretColor = color;
        turretLight.setPosition(color.position);
    }

    public void setPollenColor(Color color) {
        if (pollenLight == null || color == pollenColor) return;
        pollenColor = color;
        pollenLight.setPosition(color.position);
    }

    public Color getTurretColor() {
        return turretColor;
    }

    public Color getPollenColor() {
        return pollenColor;
    }

    @Override
    public void periodic() {
        if (turretAimed != null) setTurretColor(turretAimed.getAsBoolean() ? TURRET_AIMED_COLOR : TURRET_NOT_AIMED_COLOR);
        if (pollenPresent == null) return;

        if (pollenPresent.getAsBoolean()) {
            if (!blinking) blinkTimer.reset();
            blinking = true;
            lastSeenTimer.reset();
        } else if (blinking && lastSeenTimer.seconds() >= BLINK_TIME_S) {
            blinking = false;
        }

        if (blinking) {
            boolean on = (int) (blinkTimer.seconds() / BLINK_PERIOD_S) % 2 == 0;
            setPollenColor(on ? POLLEN_BLINK_COLOR : Color.OFF);
        } else {
            setPollenColor(POLLEN_IDLE_COLOR);
        }
    }

    public void addTelemetry(TelemetryData telemetry) {
        if (turretLight != null) telemetry.addData("Turret light", turretColor);
        if (pollenLight != null) telemetry.addData("Pollen light", blinking ? "BLINK " + POLLEN_BLINK_COLOR : pollenColor);
    }
}
