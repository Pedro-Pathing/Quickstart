package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * TauraServo
 * <p>
 * Lightweight wrapper over an FTC {@link Servo} that optionally reads an analog
 * feedback sensor (e.g., a built-in potentiometer) to expose:
 * - Commanded servo controls (position, range scaling, direction);
 * - A raw angle (degrees) converted from the analog feedback;
 * - A continuous incremental angle accumulator with unwrap across the 0° boundary.
 * <p>
 * To enable analog feedback readings, call {@link #setAnalogFeedbackSensor(AnalogInput)}
 * with a valid AnalogInput from the hardware map.
 */
public class TauraServo {

    /** Linear calibration coefficients that map sensor voltage to a normalized "universal" position in [0..1]. */
    private static final double A = -0.06637714;
    private static final double B =  0.34132813;

    /** Maximum mechanical/angular travel (degrees) used to convert universal [0..1] into raw degrees. */
    private static final double MAX_ANGLE_DEGREES_ANGULAR = 315;
    /**
     * Effective span (degrees) used for "unwrap" of the incremental measurement.
     * Choose the span that best represents one full cycle of your analog sensor (e.g., ~330° for many pots).
     */
    private static final double MAX_ANGLE_DEGREES_CONTINOUS = 330;

    /** Last sampled raw angle (degrees) used to compute deltas between samples. */
    private double lastRawAngleInDegrees = 0.0;
    /** Accumulated continuous displacement (degrees) since start (or since last implicit reset via restart). */
    private double deltaPositionInDegrees = 0.0;
    /** Optional analog input used as feedback source; if null, read functions return NaN. */
    private AnalogInput analogFeedbackSensor;

    /** Underlying servo instance obtained from the hardware map. */
    private Servo baseServo = null;

    /**
     * Constructor.
     * @param base the base Servo instance, typically retrieved via hardwareMap.get(Servo.class, "name")
     */
    public TauraServo(@NonNull Servo base) {
        baseServo = base;
    }

    /**
     * Sets the target servo position in [0..1].
     * 0.0 maps to the configured min pulse; 1.0 maps to the configured max pulse.
     */
    public void setPosition(double position) { baseServo.setPosition(position); }

    /**
     * Returns the last target position sent to the servo in [0..1].
     */
    public double getPosition() { return baseServo.getPosition(); }

    /**
     * Scales the logical range of motion without changing the servo type in Robot Configuration.
     * Useful to constrain movement to a safer subrange.
     */
    public void scaleRange(double min, double max) { baseServo.scaleRange(min, max); }

    /**
     * Sets the servo direction (FORWARD/REVERSE), effectively mirroring the logical [0..1] range.
     */
    public void setDirection(Servo.Direction direction) { baseServo.setDirection(direction); }

    /**
     * Assigns the analog feedback sensor. Without this, feedback-based getters will return NaN.
     */
    public void setAnalogFeedbackSensor(AnalogInput analogFeedbackSensor) {
        this.analogFeedbackSensor = analogFeedbackSensor;
    }

    /**
     * Returns the continuous incremental displacement in degrees.
     * Applies "unwrap" across the 0° boundary so small real movements near wrap-around are accumulated correctly.
     * If the analog reading is unavailable (NaN), the accumulator is preserved and returned unchanged.
     * <p>
     * Use for Servo in Continous mode.
     */
    public double getIncrementalPositionInDegrees() {
        double actualAngleInDegrees = getRawPositionInDegrees();

        double delta = actualAngleInDegrees - lastRawAngleInDegrees;
        double halfRange = MAX_ANGLE_DEGREES_CONTINOUS / 2.0;

        // Unwrap: if the jump crosses the wrap boundary, choose the shortest equivalent delta
        if (delta >  halfRange) delta -= MAX_ANGLE_DEGREES_CONTINOUS;
        else if (delta < -halfRange) delta += MAX_ANGLE_DEGREES_CONTINOUS;

        deltaPositionInDegrees += delta;
        lastRawAngleInDegrees = actualAngleInDegrees;

        return deltaPositionInDegrees;
    }

    /**
     * Returns the current raw angle (degrees) within the expected angular span.
     * Converts the normalized universal position [0..1] to degrees using MAX_ANGLE_DEGREES_ANGULAR.
     * If the universal position is NaN, returns NaN.
     */
    public double getRawPositionInDegrees() {
        return getUniversalPosition() * MAX_ANGLE_DEGREES_ANGULAR;
    }

    /**
     * Returns the normalized "universal" position in [0..1] computed from the analog voltage
     * using the linear calibration model A + B * voltage. If no sensor is set, returns NaN.
     */
    public double getUniversalPosition() {
        return A + B * getVoltageFromAnalogSensor();
    }

    /**
     * Reads the analog sensor voltage. If no sensor is configured, returns NaN instead of throwing.
     */
    private double getVoltageFromAnalogSensor() {
        try {
            return analogFeedbackSensor.getVoltage();
        } catch(NullPointerException e) {
            return Double.NaN;
        }
    }

}
