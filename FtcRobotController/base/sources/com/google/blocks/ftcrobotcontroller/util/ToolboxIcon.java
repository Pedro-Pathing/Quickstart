package com.google.blocks.ftcrobotcontroller.util;

/* JADX INFO: loaded from: classes8.dex */
public enum ToolboxIcon {
    ACCELERATION_SENSOR("AccelerationSensor-icon"),
    ANALOG_INPUT("AnalogInput-icon"),
    COLOR_SENSOR("ColorSensor-icon"),
    COMPASS_SENSOR("CompassSensor-icon"),
    CR_SERVO("CRServo-icon"),
    DC_MOTOR("DcMotor-icon"),
    DIGITAL_CHANNEL("DigitalChannel-icon"),
    ELAPSED_TIME("ElapsedTime-icon"),
    GAMEPAD("Gamepad-icon"),
    GYRO_SENSOR("GyroSensor-icon"),
    IR_SEEKER_SENSOR("IrSeekerSensor-icon"),
    LED("LED-icon"),
    LIGHT_SENSOR("LightSensor-icon"),
    LINEAR_OPMODE("LinearOpMode-icon"),
    OCTOQUAD("OctoQuad-icon"),
    OPTICAL_DISTANCE_SENSOR("OpticalDistanceSensor-icon"),
    SERVO("Servo-icon"),
    SERVO_CONTROLLER("ServoController-icon"),
    TOUCH_SENSOR("TouchSensor-icon"),
    ULTRASONIC_SENSOR("UltrasonicSensor-icon"),
    VOLTAGE_SENSOR("VoltageSensor-icon");

    public final String cssClass;

    ToolboxIcon(String cssClass) {
        this.cssClass = cssClass;
    }
}
