package com.google.blocks.ftcrobotcontroller.hardware;

import com.google.blocks.ftcrobotcontroller.util.ToolboxFolder;
import com.google.blocks.ftcrobotcontroller.util.ToolboxIcon;
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.andymark.AndyMarkColorSensor;
import com.qualcomm.hardware.andymark.AndyMarkTOF;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuadImpl;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.maxbotix.MaxSonarI2CXL;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.hardware.sparkfun.SparkFunLEDStick;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.AccelerationSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.BuiltInConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.ConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.ConfigurationTypeManager;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/* JADX INFO: loaded from: classes8.dex */
public enum HardwareType {
    ACCELERATION_SENSOR("createAccelerationSensorDropdown", "accelerationSensor", "AsAccelerationSensor", "_AccelerationSensor", ToolboxFolder.SENSORS, "AccelerationSensor", ToolboxIcon.ACCELERATION_SENSOR, AccelerationSensor.class, BuiltInConfigurationType.ACCELEROMETER.getXmlTag()),
    ANALOG_INPUT("createAnalogInputDropdown", "analogInput", "AsAnalogInput", "_AnalogInput", ToolboxFolder.OTHER, "AnalogInput", ToolboxIcon.ANALOG_INPUT, AnalogInput.class, ConfigurationTypeManager.getXmlTag(AnalogInput.class)),
    ANDY_MARK_COLOR_SENSOR("createAndyMarkColorSensorDropdown", "andyMarkColorSensor", "AsAndyMarkColorSensor", "_AndyMarkColorSensor", ToolboxFolder.SENSORS, "AndyMarkColorSensor", ToolboxIcon.COLOR_SENSOR, AndyMarkColorSensor.class, ConfigurationTypeManager.getXmlTag(AndyMarkColorSensor.class)),
    BNO055IMU("createBNO055IMUDropdown", "bno055imu", "AsBNO055IMU", "_IMU_BNO055", ToolboxFolder.SENSORS, "IMU-BNO055 (legacy)", null, BNO055IMUImpl.class, ConfigurationTypeManager.getXmlTag(AdafruitBNO055IMU.class), ConfigurationTypeManager.getXmlTag(LynxEmbeddedIMU.class)),
    COLOR_RANGE_SENSOR("createColorRangeSensorDropdown", "lynxI2cColorRangeSensor", "AsREVColorRangeSensor", "_REV_ColorRangeSensor", ToolboxFolder.SENSORS, "REV Color/Range Sensor", ToolboxIcon.COLOR_SENSOR, ColorRangeSensor.class, BuiltInConfigurationType.LYNX_COLOR_SENSOR.getXmlTag(), ConfigurationTypeManager.getXmlTag(RevColorSensorV3.class)),
    COLOR_SENSOR("createColorSensorDropdown", "colorSensor", "AsColorSensor", "_ColorSensor", ToolboxFolder.SENSORS, "ColorSensor", ToolboxIcon.COLOR_SENSOR, ColorSensor.class, BuiltInConfigurationType.COLOR_SENSOR.getXmlTag(), BuiltInConfigurationType.ADAFRUIT_COLOR_SENSOR.getXmlTag(), BuiltInConfigurationType.LYNX_COLOR_SENSOR.getXmlTag(), ConfigurationTypeManager.getXmlTag(RevColorSensorV3.class)),
    COMPASS_SENSOR("createCompassSensorDropdown", "compassSensor", "AsCompassSensor", "_CompassSensor", ToolboxFolder.SENSORS, "CompassSensor", ToolboxIcon.COMPASS_SENSOR, CompassSensor.class, BuiltInConfigurationType.COMPASS.getXmlTag()),
    CR_SERVO("createCRServoDropdown", "crServo", "AsCRServo", "_CRServo", ToolboxFolder.ACTUATORS, "CRServo", ToolboxIcon.CR_SERVO, CRServo.class, getContinuousServoXmlTags()),
    DC_MOTOR("createDcMotorDropdown", "dcMotor", "AsDcMotor", "_DcMotor", ToolboxFolder.ACTUATORS, "DcMotor", ToolboxIcon.DC_MOTOR, DcMotor.class, getMotorXmlTags()),
    DIGITAL_CHANNEL("createDigitalChannelDropdown", "digitalChannel", "AsDigitalChannel", "_DigitalChannel", ToolboxFolder.OTHER, "DigitalChannel", ToolboxIcon.DIGITAL_CHANNEL, DigitalChannel.class, ConfigurationTypeManager.getXmlTag(DigitalChannelImpl.class)),
    DISTANCE_SENSOR("createDistanceSensorDropdown", "distanceSensor", "AsDistanceSensor", "_DistanceSensor", ToolboxFolder.SENSORS, "DistanceSensor", ToolboxIcon.ULTRASONIC_SENSOR, DistanceSensor.class, BuiltInConfigurationType.LYNX_COLOR_SENSOR.getXmlTag(), ConfigurationTypeManager.getXmlTag(AndyMarkTOF.class), ConfigurationTypeManager.getXmlTag(Rev2mDistanceSensor.class), ConfigurationTypeManager.getXmlTag(RevColorSensorV3.class)),
    GOBILDA_PINPOINT("createGoBildaPinpointDropdown", "goBildaPinpoint", "AsGoBildaPinpoint", "_GoBildaPinpoint", ToolboxFolder.SENSORS, "GoBildaPinpoint", null, GoBildaPinpointDriver.class, ConfigurationTypeManager.getXmlTag(GoBildaPinpointDriver.class)),
    GYRO_SENSOR("createGyroSensorDropdown", "gyroSensor", "AsGyroSensor", "_GyroSensor", ToolboxFolder.SENSORS, "GyroSensor", ToolboxIcon.GYRO_SENSOR, GyroSensor.class, BuiltInConfigurationType.GYRO.getXmlTag()),
    HUSKY_LENS("createHuskyLensDropdown", "huskyLens", "AsHuskyLens", "_HuskyLens", ToolboxFolder.SENSORS, "HuskyLens", null, HuskyLens.class, ConfigurationTypeManager.getXmlTag(HuskyLens.class)),
    IMU("createImuDropdown", "imu", "AsIMU", "_IMU", ToolboxFolder.SENSORS, "IMU", null, IMU.class, LynxConstants.EMBEDDED_BNO055_IMU_XML_TAG, LynxConstants.EMBEDDED_BHI260AP_IMU_XML_TAG, ConfigurationTypeManager.getXmlTag(AdafruitBNO055IMU.class)),
    IR_SEEKER_SENSOR("createIrSeekerSensorDropdown", "irSeekerSensor", "AsIrSeekerSensor", "_IrSeekerSensor", ToolboxFolder.SENSORS, "IrSeekerSensor", ToolboxIcon.IR_SEEKER_SENSOR, IrSeekerSensor.class, BuiltInConfigurationType.IR_SEEKER.getXmlTag(), BuiltInConfigurationType.IR_SEEKER_V3.getXmlTag()),
    LED("createLedDropdown", "led", "AsLED", "_LED", ToolboxFolder.OTHER, "LED", ToolboxIcon.LED, LED.class, ConfigurationTypeManager.getXmlTag(LED.class)),
    LIGHT_SENSOR("createLightSensorDropdown", "lightSensor", "AsLightSensor", "_LightSensor", ToolboxFolder.SENSORS, "LightSensor", ToolboxIcon.LIGHT_SENSOR, LightSensor.class, BuiltInConfigurationType.LIGHT_SENSOR.getXmlTag()),
    LIMELIGHT_3A("createLimelight3ADropdown", "limelight3A", "AsLimelight3A", "_Limelight3A", ToolboxFolder.SENSORS, "Limelight3A", null, Limelight3A.class, BuiltInConfigurationType.ETHERNET_OVER_USB_DEVICE.getXmlTag()),
    LYNX_MODULE(null, null, "AsREVModule", "_REV_Module", null, null, null, LynxModule.class, BuiltInConfigurationType.LYNX_MODULE.getXmlTag()),
    MAX_SONAR_I2CXL("createMaxSonarI2CXLDropdown", "maxSonarI2CXL", "AsMaxSonarI2CXL", "_MaxSonarI2CXL", ToolboxFolder.SENSORS, "MaxSonarI2CXL", null, MaxSonarI2CXL.class, ConfigurationTypeManager.getXmlTag(MaxSonarI2CXL.class)),
    MR_I2C_COMPASS_SENSOR("createMrI2cCompassSensorDropdown", "mrI2cCompassSensor", "AsMrI2cCompassSensor", "_MR_I2cCompassSensor", ToolboxFolder.SENSORS, "MrI2cCompassSensor", ToolboxIcon.COMPASS_SENSOR, ModernRoboticsI2cCompassSensor.class, ConfigurationTypeManager.getXmlTag(ModernRoboticsI2cCompassSensor.class)),
    MR_I2C_RANGE_SENSOR("createMrI2cRangeSensorDropdown", "mrI2cRangeSensor", "AsMrI2cRangeSensor", "_MR_I2cRangeSensor", ToolboxFolder.SENSORS, "MrI2cRangeSensor", ToolboxIcon.OPTICAL_DISTANCE_SENSOR, ModernRoboticsI2cRangeSensor.class, ConfigurationTypeManager.getXmlTag(ModernRoboticsI2cRangeSensor.class)),
    OCTOQUAD("createOctoQuadDropdown", "octoquad", "AsOctoQuad", "_OctoQuad", ToolboxFolder.SENSORS, "OctoQuad", ToolboxIcon.OCTOQUAD, OctoQuad.class, ConfigurationTypeManager.getXmlTag(OctoQuadImpl.class)),
    OPTICAL_DISTANCE_SENSOR("createOpticalDistanceSensorDropdown", "opticalDistanceSensor", "AsOpticalDistanceSensor", "_OpticalDistanceSensor", ToolboxFolder.SENSORS, "OpticalDistanceSensor", ToolboxIcon.OPTICAL_DISTANCE_SENSOR, OpticalDistanceSensor.class, ConfigurationTypeManager.getXmlTag(ModernRoboticsAnalogOpticalDistanceSensor.class), BuiltInConfigurationType.LYNX_COLOR_SENSOR.getXmlTag(), ConfigurationTypeManager.getXmlTag(RevColorSensorV3.class)),
    REV_BLINKIN_LED_DRIVER("createRevBlinkinLedDriverDropdown", "revBlinkinLedDriver", "AsRevBlinkinLedDriver", "_RevBlinkinLedDriver", ToolboxFolder.OTHER, "RevBlinkinLedDriver", ToolboxIcon.LED, RevBlinkinLedDriver.class, ConfigurationTypeManager.getXmlTag(RevBlinkinLedDriver.class)),
    SERVO("createServoDropdown", "servo", "AsServo", "_Servo", ToolboxFolder.ACTUATORS, "Servo", ToolboxIcon.SERVO, Servo.class, getStandardServoXmlTags()),
    SERVO_CONTROLLER("createServoControllerDropdown", "servoController", "AsServoController", "_ServoController", ToolboxFolder.ACTUATORS, "ServoController", ToolboxIcon.SERVO_CONTROLLER, ServoController.class, BuiltInConfigurationType.LYNX_MODULE.getXmlTag()),
    SPARKFUN_LED_STICK("createSparkFunLEDStickDropdown", "sparkFunLEDStick", "AsSparkFunLEDStick", "_SparkFunLEDStick", ToolboxFolder.OTHER, "SparkFunLEDStick", null, SparkFunLEDStick.class, ConfigurationTypeManager.getXmlTag(SparkFunLEDStick.class)),
    SPARKFUN_OTOS("createSparkFunOTOSDropdown", "sparkFunOTOS", "AsSparkFunOTOS", "_SparkFunOTOS", ToolboxFolder.SENSORS, "SparkFunOTOS", null, SparkFunOTOS.class, ConfigurationTypeManager.getXmlTag(SparkFunOTOS.class)),
    TOUCH_SENSOR("createTouchSensorDropdown", "touchSensor", "AsTouchSensor", "_TouchSensor", ToolboxFolder.SENSORS, "TouchSensor", ToolboxIcon.TOUCH_SENSOR, TouchSensor.class, ConfigurationTypeManager.getXmlTag(ModernRoboticsTouchSensor.class), ConfigurationTypeManager.getXmlTag(RevTouchSensor.class)),
    ULTRASONIC_SENSOR("createUltrasonicSensorDropdown", "ultrasonicSensor", "AsUltrasonicSensor", "_UltrasonicSensor", ToolboxFolder.SENSORS, "UltrasonicSensor", ToolboxIcon.ULTRASONIC_SENSOR, UltrasonicSensor.class, BuiltInConfigurationType.ULTRASONIC_SENSOR.getXmlTag()),
    VOLTAGE_SENSOR("createVoltageSensorDropdown", "voltageSensor", "AsVoltageSensor", "_VoltageSensor", ToolboxFolder.SENSORS, "VoltageSensor", ToolboxIcon.VOLTAGE_SENSOR, VoltageSensor.class, BuiltInConfigurationType.LYNX_MODULE.getXmlTag()),
    WEBCAM_NAME(null, null, "AsWebcamName", "_WebcamName", null, null, null, WebcamName.class, BuiltInConfigurationType.WEBCAM.getXmlTag());

    static final Comparator<HardwareType> BY_TOOLBOX_CATEGORY_NAME = new Comparator<HardwareType>() { // from class: com.google.blocks.ftcrobotcontroller.hardware.HardwareType.1
        @Override // java.util.Comparator
        public int compare(HardwareType h1, HardwareType h2) {
            String s1 = h1.toolboxCategoryName == null ? "" : h1.toolboxCategoryName;
            String s2 = h2.toolboxCategoryName != null ? h2.toolboxCategoryName : "";
            return s1.compareToIgnoreCase(s2);
        }
    };
    public final String blockTypePrefix;
    public final String createDropdownFunctionName;
    public final Class<? extends HardwareDevice> deviceType;
    public final String identifierSuffixForFtcJava;
    public final String identifierSuffixForJavaScript;
    public final String toolboxCategoryName;
    public final ToolboxFolder toolboxFolder;
    public final ToolboxIcon toolboxIcon;
    public final String[] xmlTags;

    private static String[] getMotorXmlTags() {
        List<String> tags = new LinkedList<>();
        for (ConfigurationType type : ConfigurationTypeManager.getInstance().getApplicableConfigTypes(ConfigurationType.DeviceFlavor.MOTOR, null, false)) {
            if (type != BuiltInConfigurationType.NOTHING) {
                tags.add(type.getXmlTag());
            }
        }
        tags.add(ConfigurationTypeManager.LEGACY_HD_HEX_MOTOR_TAG);
        String[] result = new String[tags.size()];
        return (String[]) tags.toArray(result);
    }

    private static String[] getStandardServoXmlTags() {
        List<String> tags = new LinkedList<>();
        for (ConfigurationType type : ConfigurationTypeManager.getInstance().getApplicableConfigTypes(ConfigurationType.DeviceFlavor.SERVO, null, false)) {
            if (type != BuiltInConfigurationType.NOTHING && ((ServoConfigurationType) type).getServoFlavor() == ServoFlavor.STANDARD) {
                tags.add(type.getXmlTag());
            }
        }
        String[] result = new String[tags.size()];
        return (String[]) tags.toArray(result);
    }

    private static String[] getContinuousServoXmlTags() {
        List<String> tags = new LinkedList<>();
        for (ConfigurationType type : ConfigurationTypeManager.getInstance().getApplicableConfigTypes(ConfigurationType.DeviceFlavor.SERVO, null, false)) {
            if (type != BuiltInConfigurationType.NOTHING && ((ServoConfigurationType) type).getServoFlavor() == ServoFlavor.CONTINUOUS) {
                tags.add(type.getXmlTag());
            }
        }
        String[] result = new String[tags.size()];
        return (String[]) tags.toArray(result);
    }

    HardwareType(String createDropdownFunctionName, String blockTypePrefix, String identifierSuffixForJavaScript, String identifierSuffixForFtcJava, ToolboxFolder toolboxFolder, String toolboxCategoryName, ToolboxIcon toolboxIcon, Class cls, String... xmlTags) {
        if (identifierSuffixForJavaScript == null || identifierSuffixForJavaScript.isEmpty()) {
            throw new IllegalArgumentException("identifierSuffixForJavaScript");
        }
        if (identifierSuffixForFtcJava == null || identifierSuffixForFtcJava.isEmpty()) {
            throw new IllegalArgumentException("identifierSuffixForFtcJava");
        }
        this.createDropdownFunctionName = createDropdownFunctionName;
        this.blockTypePrefix = blockTypePrefix;
        this.identifierSuffixForJavaScript = identifierSuffixForJavaScript;
        this.identifierSuffixForFtcJava = identifierSuffixForFtcJava;
        this.toolboxFolder = toolboxFolder;
        this.toolboxCategoryName = toolboxCategoryName;
        this.toolboxIcon = toolboxIcon;
        this.deviceType = cls;
        this.xmlTags = xmlTags;
    }

    boolean isContainer() {
        return this.deviceType == LynxModule.class || this.deviceType == ServoController.class;
    }

    public String makeIdentifier(String deviceName) {
        return HardwareItem.makeIdentifier(deviceName) + this.identifierSuffixForJavaScript;
    }

    public static HardwareType fromIdentifierSuffixForJavaScript(String identifierSuffixForJavaScript) {
        for (HardwareType hardwareType : values()) {
            if (hardwareType.identifierSuffixForJavaScript.equals(identifierSuffixForJavaScript)) {
                return hardwareType;
            }
        }
        return null;
    }
}
