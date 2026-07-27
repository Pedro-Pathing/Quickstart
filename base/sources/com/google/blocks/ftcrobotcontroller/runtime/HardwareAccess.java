package com.google.blocks.ftcrobotcontroller.runtime;

import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareType;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

/* JADX INFO: loaded from: classes8.dex */
abstract class HardwareAccess<DEVICE_TYPE extends HardwareDevice> extends Access {
    protected final DEVICE_TYPE hardwareDevice;
    protected final HardwareItem hardwareItem;

    protected HardwareAccess(BlocksOpMode blocksOpMode, HardwareItem hardwareItem, HardwareMap hardwareMap, Class<DEVICE_TYPE> cls) {
        String str;
        super(blocksOpMode, hardwareItem.identifier, hardwareItem.visibleName);
        this.hardwareItem = hardwareItem;
        DEVICE_TYPE device_type = null;
        try {
            device_type = (DEVICE_TYPE) ((HardwareDevice) hardwareMap.get(cls, hardwareItem.deviceName));
        } catch (Exception e) {
            try {
                hardwareMap.get(hardwareItem.deviceName);
                str = "The name \"" + hardwareItem.deviceName + "\" is present in the active configuration, but it does not correspond to a " + cls.getSimpleName() + ".";
            } catch (Exception e2) {
                str = "The name \"" + hardwareItem.deviceName + "\" is not present in the active configuration.";
            }
            reportHardwareError(str);
        }
        this.hardwareDevice = device_type;
    }

    static HardwareAccess newHardwareAccess(BlocksOpMode blocksOpMode, HardwareType hardwareType, HardwareMap hardwareMap, HardwareItem hardwareItem) {
        switch (hardwareType) {
            case ACCELERATION_SENSOR:
                return new AccelerationSensorAccess(blocksOpMode, hardwareItem, hardwareMap);
            case ANALOG_INPUT:
                return new AnalogInputAccess(blocksOpMode, hardwareItem, hardwareMap);
            case ANDY_MARK_COLOR_SENSOR:
                return new AndyMarkColorSensorAccess(blocksOpMode, hardwareItem, hardwareMap);
            case BNO055IMU:
                return new BNO055IMUAccess(blocksOpMode, hardwareItem, hardwareMap);
            case COLOR_RANGE_SENSOR:
                return new ColorRangeSensorAccess(blocksOpMode, hardwareItem, hardwareMap);
            case COLOR_SENSOR:
                return new ColorSensorAccess(blocksOpMode, hardwareItem, hardwareMap);
            case COMPASS_SENSOR:
                return new CompassSensorAccess(blocksOpMode, hardwareItem, hardwareMap);
            case CR_SERVO:
                return new CRServoAccess(blocksOpMode, hardwareItem, hardwareMap);
            case DC_MOTOR:
                return new DcMotorAccess(blocksOpMode, hardwareItem, hardwareMap);
            case DIGITAL_CHANNEL:
                return new DigitalChannelAccess(blocksOpMode, hardwareItem, hardwareMap);
            case DISTANCE_SENSOR:
                return new DistanceSensorAccess(blocksOpMode, hardwareItem, hardwareMap);
            case GOBILDA_PINPOINT:
                return new GoBildaPinpointAccess(blocksOpMode, hardwareItem, hardwareMap);
            case GYRO_SENSOR:
                return new GyroSensorAccess(blocksOpMode, hardwareItem, hardwareMap);
            case HUSKY_LENS:
                return new HuskyLensAccess(blocksOpMode, hardwareItem, hardwareMap);
            case IMU:
                return new ImuAccess(blocksOpMode, hardwareItem, hardwareMap);
            case IR_SEEKER_SENSOR:
                return new IrSeekerSensorAccess(blocksOpMode, hardwareItem, hardwareMap);
            case LED:
                return new LedAccess(blocksOpMode, hardwareItem, hardwareMap);
            case LIMELIGHT_3A:
                return new Limelight3AAccess(blocksOpMode, hardwareItem, hardwareMap);
            case LIGHT_SENSOR:
                return new LightSensorAccess(blocksOpMode, hardwareItem, hardwareMap);
            case LYNX_MODULE:
                return null;
            case MAX_SONAR_I2CXL:
                return new MaxSonarI2CXLAccess(blocksOpMode, hardwareItem, hardwareMap);
            case MR_I2C_COMPASS_SENSOR:
                return new MrI2cCompassSensorAccess(blocksOpMode, hardwareItem, hardwareMap);
            case MR_I2C_RANGE_SENSOR:
                return new MrI2cRangeSensorAccess(blocksOpMode, hardwareItem, hardwareMap);
            case OCTOQUAD:
                return new OctoQuadAccess(blocksOpMode, hardwareItem, hardwareMap);
            case OPTICAL_DISTANCE_SENSOR:
                return new OpticalDistanceSensorAccess(blocksOpMode, hardwareItem, hardwareMap);
            case REV_BLINKIN_LED_DRIVER:
                return new RevBlinkinLedDriverAccess(blocksOpMode, hardwareItem, hardwareMap);
            case SERVO:
                return new ServoAccess(blocksOpMode, hardwareItem, hardwareMap);
            case SERVO_CONTROLLER:
                return new ServoControllerAccess(blocksOpMode, hardwareItem, hardwareMap);
            case SPARKFUN_LED_STICK:
                return new SparkFunLEDStickAccess(blocksOpMode, hardwareItem, hardwareMap);
            case SPARKFUN_OTOS:
                return new SparkFunOTOSAccess(blocksOpMode, hardwareItem, hardwareMap);
            case TOUCH_SENSOR:
                return new TouchSensorAccess(blocksOpMode, hardwareItem, hardwareMap);
            case ULTRASONIC_SENSOR:
                return new UltrasonicSensorAccess(blocksOpMode, hardwareItem, hardwareMap);
            case VOLTAGE_SENSOR:
                return new VoltageSensorAccess(blocksOpMode, hardwareItem, hardwareMap);
            case WEBCAM_NAME:
                return null;
            default:
                throw new IllegalArgumentException("Unknown hardware type " + hardwareType);
        }
    }
}
