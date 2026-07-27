package com.google.blocks.ftcrobotcontroller.hardware;

import android.content.Context;
import android.content.res.AssetManager;
import android.graphics.Color;
import android.hardware.SensorManager;
import androidx.core.app.NotificationCompat;
import androidx.core.internal.view.SupportMenu;
import androidx.core.view.ViewCompat;
import com.google.blocks.ftcrobotcontroller.util.AvailableTtsLocalesProvider;
import com.google.blocks.ftcrobotcontroller.util.CurrentGame;
import com.google.blocks.ftcrobotcontroller.util.FileUtil;
import com.google.blocks.ftcrobotcontroller.util.Identifier;
import com.google.blocks.ftcrobotcontroller.util.ProjectsUtil;
import com.google.blocks.ftcrobotcontroller.util.ToolboxFolder;
import com.google.blocks.ftcrobotcontroller.util.ToolboxIcon;
import com.google.blocks.ftcrobotcontroller.util.ToolboxUtil;
import com.qualcomm.ftccommon.R;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.ftccommon.configuration.RobotConfigFile;
import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;
import com.qualcomm.hardware.andymark.AndyMarkColorSensor;
import com.qualcomm.hardware.andymark.AndyMarkIMU;
import com.qualcomm.hardware.andymark.AndyMarkIMUOrientationOnRobot;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.maxbotix.MaxSonarI2CXL;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev9AxisImuOrientationOnRobot;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunLEDStick;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AccelerationSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cAddrConfig;
import com.qualcomm.robotcore.hardware.I2cAddressableDevice;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.OrientationSensor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.DeviceConfiguration;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.util.Device;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.SortOrder;
import dk.sgjesse.r8api.DescriptorUtils;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.SortedMap;
import java.util.SortedSet;
import java.util.TreeMap;
import java.util.TreeSet;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.onbotjava.RequestConditions;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.ExportAprilTagLibraryToBlocks;
import org.firstinspires.ftc.robotcore.external.ExportEnumToBlocks;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.android.AndroidAccelerometer;
import org.firstinspires.ftc.robotcore.external.android.AndroidGyroscope;
import org.firstinspires.ftc.robotcore.external.android.AndroidOrientation;
import org.firstinspires.ftc.robotcore.external.android.AndroidSoundPool;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.MagneticFlux;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Temperature;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.robotcore.internal.opmode.BlocksClassFilter;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.robotcore.internal.opmode.RegisteredOpModes;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;

/* JADX INFO: loaded from: classes8.dex */
public class HardwareUtil {
    private static final String COLOR_CATEGORY_NAME = "Color";
    private static final String DC_MOTOR_DUAL_CATEGORY_NAME = "Dual";
    private static final String DC_MOTOR_EX_CATEGORY_NAME = "Extended";
    private static final String ELAPSED_TIME_CATEGORY_NAME = "ElapsedTime";
    private static final String GAMEPAD_CATEGORY_NAME = "Gamepad";
    public static final String IDENTIFIERS_USED_PREFIX = "// IDENTIFIERS_USED=";
    private static final String LINEAR_OP_MODE_CATEGORY_NAME = "LinearOpMode";
    private static final SensorManager sensorManager = (SensorManager) AppUtil.getDefContext().getSystemService("sensor");
    private static final BlocksClassFilter blocksClassFilter = BlocksClassFilter.getInstance();
    private static final Set<String> RESERVED_WORDS_FOR_FTCJAVA = new TreeSet();
    private static final Map<String, Class<?>[]> KNOWN_TYPES_FOR_FTCJAVA = new TreeMap();
    private static final Map<String, List<HardwareType>> XML_TAG_TO_HARDWARE_TYPES = new HashMap();

    static {
        buildKnownTypesAndReservedWordsForFtcJava();
        for (HardwareType hardwareType : HardwareType.values()) {
            for (String xmlTag : hardwareType.xmlTags) {
                List<HardwareType> list = XML_TAG_TO_HARDWARE_TYPES.get(xmlTag);
                if (list == null) {
                    list = new ArrayList();
                    XML_TAG_TO_HARDWARE_TYPES.put(xmlTag, list);
                }
                list.add(hardwareType);
            }
        }
    }

    public enum Capability {
        BUILTIN_CAMERA("builtinCamera"),
        WEBCAM("webcam"),
        SWITCHABLE_CAMERA("switchableCamera"),
        VISION("vision");

        private final String placeholderType;

        Capability(String placeholderType) {
            this.placeholderType = placeholderType;
        }

        static Capability fromPlaceholderType(String type) {
            for (Capability capability : values()) {
                if (capability.placeholderType.equals(type)) {
                    return capability;
                }
            }
            throw new IllegalArgumentException("Unexpected capability name " + type);
        }
    }

    private HardwareUtil() {
    }

    static Iterable<HardwareType> getHardwareTypes(String xmlTag) {
        if (XML_TAG_TO_HARDWARE_TYPES.containsKey(xmlTag)) {
            return Collections.unmodifiableList(XML_TAG_TO_HARDWARE_TYPES.get(xmlTag));
        }
        return Collections.emptyList();
    }

    static Iterable<HardwareType> getHardwareTypes(DeviceConfiguration deviceConfiguration) {
        return getHardwareTypes(deviceConfiguration.getConfigurationType().getXmlTag());
    }

    public static String fetchJavaScriptForHardware() throws IOException {
        return fetchJavaScriptForHardware(HardwareItemMap.newHardwareItemMap());
    }

    public static String fetchJavaScriptForHardware(HardwareItemMap hardwareItemMap) throws Throwable {
        Map<Capability, Boolean> capabilities;
        String warning;
        Class<?>[] clsArr;
        int i;
        StringBuilder createSkyStoneSoundResourceDropdown;
        Iterator<Map.Entry<String, String>> it;
        String defaultLanguageCode;
        Context context = AppUtil.getDefContext();
        AssetManager assetManager = context.getAssets();
        StringBuilder jsHardware = new StringBuilder().append("\n");
        Map<Capability, Boolean> capabilities2 = getCapabilities(hardwareItemMap);
        Set<String> additionalReservedWordsForFtcJava = new HashSet<>();
        Set<String> methodLookupStrings = new HashSet<>();
        String toolbox = generateToolbox(hardwareItemMap, capabilities2, assetManager, additionalReservedWordsForFtcJava, methodLookupStrings, jsHardware);
        String toolbox2 = toolbox.replace("\n", " ").replaceAll("\\> +\\<", "><");
        Set<String> teleOpNames = new TreeSet<>();
        RegisteredOpModes registeredOpModes = RegisteredOpModes.getInstance();
        registeredOpModes.waitOpModesRegistered();
        for (OpModeMeta opModeMeta : registeredOpModes.getOpModes()) {
            if (opModeMeta.flavor == OpModeMeta.Flavor.TELEOP) {
                teleOpNames.add(opModeMeta.name);
            }
        }
        jsHardware.append("var allHardwareIdentifiers = [\n");
        Iterator<HardwareItem> it2 = hardwareItemMap.getAllHardwareItems().iterator();
        while (it2.hasNext()) {
            jsHardware.append("    '").append(it2.next().identifier).append("',\n");
        }
        jsHardware.append("  ];\n\n");
        jsHardware.append("var IDENTIFIERS_USED_PREFIX = '").append(IDENTIFIERS_USED_PREFIX).append("';\n\n");
        jsHardware.append("var XML_END_TAG = '").append(ProjectsUtil.escapeSingleQuotes(ProjectsUtil.XML_END_TAG)).append("';\n").append("var XML_EXTRA_START = '").append(ProjectsUtil.escapeSingleQuotes(ProjectsUtil.XML_EXTRA_START)).append("';\n\n");
        jsHardware.append("var AUTO_TRANSITION_OPTIONS = [\n");
        for (String teleOpName : teleOpNames) {
            jsHardware.append("  '").append(ProjectsUtil.escapeSingleQuotes(teleOpName)).append("',\n");
            teleOpNames = teleOpNames;
            registeredOpModes = registeredOpModes;
        }
        jsHardware.append("];\n\n");
        jsHardware.append("var currentGameName = '").append(ProjectsUtil.escapeSingleQuotes(CurrentGame.CURRENT_GAME_NAME)).append("';\n").append("var currentGameNameNoSpaces = '").append(ProjectsUtil.escapeSingleQuotes(CurrentGame.CURRENT_GAME_NAME_NO_SPACES)).append("';\n").append("\n");
        jsHardware.append("var methodLookupStrings = [\n");
        for (String methodLookupString : methodLookupStrings) {
            jsHardware.append("  '").append(methodLookupString).append("',\n");
        }
        jsHardware.append("];\n\n");
        String str = "  return '';\n";
        jsHardware.append("function getOctoQuadConstant(constantIdentifier) {\n").append("  switch (constantIdentifier) {\n").append("    case 'OCTOQUAD_CHIP_ID':\n").append("      return '").append("0x").append(Integer.toHexString(81).toUpperCase()).append("';\n").append("    case 'SUPPORTED_FW_VERSION_MAJ':\n").append("      return '").append(3).append("';\n").append("    case 'ENCODER_FIRST':\n").append("      return '").append(0).append("';\n").append("    case 'ENCODER_LAST':\n").append("      return '").append(7).append("';\n").append("    case 'NUM_ENCODERS':\n").append("      return '").append(8).append("';\n").append("    case 'MIN_VELOCITY_MEASUREMENT_INTERVAL_MS':\n").append("      return '").append(1).append("';\n").append("    case 'MAX_VELOCITY_MEASUREMENT_INTERVAL_MS':\n").append("      return '").append(255).append("';\n").append("    case 'MIN_PULSE_WIDTH_US':\n").append("      return '").append(0).append("';\n").append("    case 'MAX_PULSE_WIDTH_US':\n").append("      return '").append("0x").append(Integer.toHexString(65535).toUpperCase()).append("';\n").append("  }\n").append("  return '';\n").append("}\n\n");
        jsHardware.append("function isValidProjectName(projectName) {\n").append("  if (projectName) {\n").append("    return /").append(ProjectsUtil.VALID_PROJECT_REGEX).append("/.test(projectName);\n").append("  }\n").append("  return false;\n").append("}\n\n");
        String str2 = "    '";
        jsHardware.append("function getSparkFunOTOSConstant(constantIdentifier) {\n").append("  switch (constantIdentifier) {\n").append("    case 'MIN_SCALAR':\n").append("      return '").append(0.872d).append("';\n").append("    case 'MAX_SCALAR':\n").append("      return '").append(1.127d).append("';\n").append("  }\n").append("  return '';\n").append("}\n\n");
        jsHardware.append("function getColorConstant(constantIdentifier) {\n").append("  switch (constantIdentifier) {\n").append("    case 'BLACK':\n").append("      return '").append("0x").append(Integer.toHexString(ViewCompat.MEASURED_STATE_MASK).toUpperCase()).append("';\n").append("    case 'BLUE':\n").append("      return '").append("0x").append(Integer.toHexString(-16776961).toUpperCase()).append("';\n").append("    case 'CYAN':\n").append("      return '").append("0x").append(Integer.toHexString(-16711681).toUpperCase()).append("';\n").append("    case 'DKGRAY':\n").append("      return '").append("0x").append(Integer.toHexString(-12303292).toUpperCase()).append("';\n").append("    case 'GRAY':\n").append("      return '").append("0x").append(Integer.toHexString(-7829368).toUpperCase()).append("';\n").append("    case 'GREEN':\n").append("      return '").append("0x").append(Integer.toHexString(-16711936).toUpperCase()).append("';\n").append("    case 'LTGRAY':\n").append("      return '").append("0x").append(Integer.toHexString(-3355444).toUpperCase()).append("';\n").append("    case 'MAGENTA':\n").append("      return '").append("0x").append(Integer.toHexString(-65281).toUpperCase()).append("';\n").append("    case 'RED':\n").append("      return '").append("0x").append(Integer.toHexString(SupportMenu.CATEGORY_MASK).toUpperCase()).append("';\n").append("    case 'WHITE':\n").append("      return '").append("0x").append(Integer.toHexString(-1).toUpperCase()).append("';\n").append("    case 'YELLOW':\n").append("      return '").append("0x").append(Integer.toHexString(-256).toUpperCase()).append("';\n").append("  }\n").append("  return '';\n").append("}\n\n");
        StringBuilder blinkinPatternTooltips = new StringBuilder();
        StringBuilder blinkinPatternFromTextTooltip = new StringBuilder();
        blinkinPatternTooltips.append("var BLINKIN_PATTERN_TOOLTIPS = [\n");
        blinkinPatternFromTextTooltip.append("var BLINKIN_PATTERN_FROM_TEXT_TOOLTIP =\n").append("    'Returns the pattern associated with the given text. Valid input is ' +\n");
        RevBlinkinLedDriver.BlinkinPattern[] blinkinPatterns = RevBlinkinLedDriver.BlinkinPattern.values();
        RevBlinkinLedDriver.BlinkinPattern blinkinPattern = blinkinPatterns[0];
        int i2 = 0;
        while (true) {
            capabilities = capabilities2;
            if (i2 >= blinkinPatterns.length - 1) {
                break;
            }
            String str3 = str;
            blinkinPatternTooltips.append("  ['").append(blinkinPattern).append("', 'The BlinkinPattern value ").append(blinkinPattern).append(".'],\n");
            String str4 = str2;
            blinkinPatternFromTextTooltip.append(str4).append(blinkinPattern).append(", ' +\n");
            i2++;
            blinkinPattern = blinkinPatterns[i2];
            additionalReservedWordsForFtcJava = additionalReservedWordsForFtcJava;
            capabilities2 = capabilities;
            str2 = str4;
            str = str3;
        }
        String str5 = str;
        String str6 = str2;
        Set<String> additionalReservedWordsForFtcJava2 = additionalReservedWordsForFtcJava;
        blinkinPatternTooltips.append("];\n");
        blinkinPatternFromTextTooltip.append("    'or ").append(blinkinPattern).append(".';\n");
        jsHardware.append((CharSequence) blinkinPatternTooltips).append("\n").append((CharSequence) blinkinPatternFromTextTooltip).append("\n");
        SortedMap<String, String> languageCodes = new TreeMap<>();
        SortedMap<String, String> countryCodes = new TreeMap<>();
        for (Locale locale : AvailableTtsLocalesProvider.getInstance().getAvailableTtsLocales()) {
            StringBuilder blinkinPatternTooltips2 = blinkinPatternTooltips;
            StringBuilder blinkinPatternFromTextTooltip2 = blinkinPatternFromTextTooltip;
            languageCodes.put(locale.getLanguage(), locale.getDisplayLanguage());
            String countryCode = locale.getCountry();
            if (!countryCode.isEmpty()) {
                countryCodes.put(countryCode, locale.getDisplayCountry());
            }
            blinkinPatternTooltips = blinkinPatternTooltips2;
            blinkinPatternFromTextTooltip = blinkinPatternFromTextTooltip2;
        }
        Locale defaultLocale = Locale.getDefault();
        String defaultLanguageCode2 = defaultLocale.getLanguage();
        StringBuilder createLanguageCodeDropdown = new StringBuilder();
        StringBuilder languageCodeTooltips = new StringBuilder();
        createLanguageCodeDropdown.append("function createLanguageCodeDropdown() {\n").append("  var CHOICES = [\n");
        languageCodeTooltips.append("var LANGUAGE_CODE_TOOLTIPS = [\n");
        addLanguage(defaultLanguageCode2, defaultLocale.getDisplayLanguage(), createLanguageCodeDropdown, languageCodeTooltips);
        for (Map.Entry<String, String> entry : languageCodes.entrySet()) {
            SortedMap<String, String> languageCodes2 = languageCodes;
            String languageCode = entry.getKey();
            if (languageCode.equals(defaultLanguageCode2)) {
                defaultLanguageCode = defaultLanguageCode2;
            } else {
                defaultLanguageCode = defaultLanguageCode2;
                String languageName = entry.getValue();
                addLanguage(languageCode, languageName, createLanguageCodeDropdown, languageCodeTooltips);
            }
            languageCodes = languageCodes2;
            defaultLanguageCode2 = defaultLanguageCode;
        }
        createLanguageCodeDropdown.append("  ];\n").append("  return createFieldDropdown(CHOICES);\n").append("}\n\n");
        languageCodeTooltips.append("];\n");
        jsHardware.append((CharSequence) createLanguageCodeDropdown).append((CharSequence) languageCodeTooltips).append("\n");
        String defaultCountryCode = defaultLocale.getCountry();
        StringBuilder createCountryCodeDropdown = new StringBuilder();
        StringBuilder countryCodeTooltips = new StringBuilder();
        createCountryCodeDropdown.append("function createCountryCodeDropdown() {\n").append("  var CHOICES = [\n");
        countryCodeTooltips.append("var COUNTRY_CODE_TOOLTIPS = [\n");
        addCountry(defaultCountryCode, defaultLocale.getDisplayCountry(), createCountryCodeDropdown, countryCodeTooltips);
        Iterator<Map.Entry<String, String>> it3 = countryCodes.entrySet().iterator();
        while (it3.hasNext()) {
            Map.Entry<String, String> entry2 = it3.next();
            Locale defaultLocale2 = defaultLocale;
            String countryCode2 = entry2.getKey();
            if (countryCode2.equals(defaultCountryCode)) {
                it = it3;
            } else {
                it = it3;
                String countryName = entry2.getValue();
                addCountry(countryCode2, countryName, createCountryCodeDropdown, countryCodeTooltips);
            }
            defaultLocale = defaultLocale2;
            it3 = it;
        }
        createCountryCodeDropdown.append("  ];\n").append("  return createFieldDropdown(CHOICES);\n").append("}\n\n");
        countryCodeTooltips.append("];\n");
        jsHardware.append((CharSequence) createCountryCodeDropdown).append((CharSequence) countryCodeTooltips).append("\n");
        StringBuilder createSkyStoneSoundResourceDropdown2 = new StringBuilder();
        StringBuilder skyStoneSoundResourceTooltips = new StringBuilder();
        createSkyStoneSoundResourceDropdown2.append("function createSkyStoneSoundResourceDropdown() {\n").append("  var CHOICES = [\n");
        skyStoneSoundResourceTooltips.append("var SKY_STONE_SOUND_RESOURCE_TOOLTIPS = [\n");
        List<String> resourceNames = new ArrayList<>();
        Field[] fields = R.raw.class.getFields();
        int length = fields.length;
        int i3 = 0;
        while (i3 < length) {
            Field field = fields[i3];
            Field[] fieldArr = fields;
            String resourceName = field.getName();
            int i4 = length;
            String str7 = str6;
            if (resourceName.toUpperCase().startsWith("SS_")) {
                resourceNames.add(resourceName);
            }
            i3++;
            fields = fieldArr;
            length = i4;
            str6 = str7;
        }
        String str8 = str6;
        Collections.sort(resourceNames);
        for (String resourceName2 : resourceNames) {
            createSkyStoneSoundResourceDropdown2.append("      ['").append(ProjectsUtil.escapeSingleQuotes(makeVisibleNameForDropdownItem(resourceName2))).append("', '").append(ProjectsUtil.escapeSingleQuotes(resourceName2)).append("'],\n");
            skyStoneSoundResourceTooltips.append("  ['").append(ProjectsUtil.escapeSingleQuotes(resourceName2)).append("', 'The SoundResource value ").append(ProjectsUtil.escapeSingleQuotes(resourceName2)).append(".'],\n");
        }
        createSkyStoneSoundResourceDropdown2.append("  ];\n").append("  return createFieldDropdown(CHOICES);\n").append("}\n\n");
        skyStoneSoundResourceTooltips.append("];\n");
        jsHardware.append((CharSequence) createSkyStoneSoundResourceDropdown2).append((CharSequence) skyStoneSoundResourceTooltips).append("\n");
        jsHardware.append("var androidSoundPoolRawResPrefix = '").append(AndroidSoundPool.RAW_RES_PREFIX).append("';\n");
        Identifier[] identifierArrValues = Identifier.values();
        int length2 = identifierArrValues.length;
        int i5 = 0;
        while (i5 < length2) {
            Identifier identifier = identifierArrValues[i5];
            if (identifier.variableForJavaScript == null) {
                createSkyStoneSoundResourceDropdown = createSkyStoneSoundResourceDropdown2;
            } else {
                createSkyStoneSoundResourceDropdown = createSkyStoneSoundResourceDropdown2;
                jsHardware.append("var ").append(identifier.variableForJavaScript).append(" = '").append(identifier.identifierForJavaScript).append("';\n");
            }
            if (identifier.variableForFtcJava != null) {
                jsHardware.append("var ").append(identifier.variableForFtcJava).append(" = '").append(identifier.identifierForFtcJava).append("';\n");
            }
            i5++;
            createSkyStoneSoundResourceDropdown2 = createSkyStoneSoundResourceDropdown;
        }
        jsHardware.append("\n");
        jsHardware.append("function createWebcamDeviceNameDropdown() {\n").append("  var CHOICES = [\n");
        List<HardwareItem> hardwareItemsForWebcam = hardwareItemMap.getHardwareItems(HardwareType.WEBCAM_NAME);
        for (HardwareItem hardwareItemForWebcam : hardwareItemsForWebcam) {
            jsHardware.append("    ['").append(ProjectsUtil.escapeSingleQuotes(hardwareItemForWebcam.visibleName)).append("', '").append(ProjectsUtil.escapeSingleQuotes(hardwareItemForWebcam.deviceName)).append("'],\n");
        }
        jsHardware.append("  ];\n").append("  return createFieldDropdown(CHOICES);\n").append("}\n\n");
        for (HardwareType hardwareType : HardwareType.values()) {
            if (hardwareType.createDropdownFunctionName != null) {
                List<HardwareItem> hardwareItems = hardwareItemMap.getHardwareItems(hardwareType);
                appendCreateDropdownFunction(jsHardware, hardwareType.createDropdownFunctionName, hardwareItems);
                if (hardwareType == HardwareType.DC_MOTOR) {
                    List<HardwareItem> hardwareItemsForDcMotorEx = getHardwareItemsForDcMotorEx(hardwareItems);
                    appendCreateDropdownFunction(jsHardware, "createDcMotorExDropdown", hardwareItemsForDcMotorEx);
                }
            }
        }
        jsHardware.append("function getHardwareIdentifierSuffixes() {\n").append("  var suffixes = [\n");
        for (HardwareType hardwareType2 : HardwareType.values()) {
            jsHardware.append(str8 + hardwareType2.identifierSuffixForJavaScript + "',\n");
        }
        jsHardware.append("  ];\n").append("  return suffixes;\n").append("}\n\n");
        jsHardware.append("function addReservedWordsForJavaScript() {\n");
        Iterator<HardwareItem> it4 = hardwareItemMap.getAllHardwareItems().iterator();
        while (it4.hasNext()) {
            jsHardware.append("  Blockly.JavaScript.addReservedWords('").append(it4.next().identifier).append("');\n");
        }
        List<String> identifiersForJavaScript = new ArrayList<>();
        for (Identifier identifier2 : Identifier.values()) {
            if (identifier2.identifierForJavaScript != null && !identifier2.identifierForJavaScript.isEmpty()) {
                identifiersForJavaScript.add(identifier2.identifierForJavaScript);
            }
        }
        Collections.sort(identifiersForJavaScript);
        for (String identifierForJavaScript : identifiersForJavaScript) {
            jsHardware.append("  Blockly.JavaScript.addReservedWords('").append(identifierForJavaScript).append("');\n");
        }
        jsHardware.append("}\n\n");
        jsHardware.append("function getHardwareItemDeviceName(identifier) {\n").append("  switch (identifier) {\n");
        for (HardwareItem hardwareItem : hardwareItemMap.getAllHardwareItems()) {
            jsHardware.append("    case '").append(hardwareItem.identifier).append("':\n").append("      return '").append(ProjectsUtil.escapeSingleQuotes(hardwareItem.deviceName)).append("';\n");
        }
        jsHardware.append("  }\n").append("  throw 'Unexpected identifier (' + identifier + ').';\n").append("}\n\n");
        jsHardware.append("function getHardwareItemIdentifierForFtcJava(identifier) {\n").append("  switch (identifier) {\n");
        Set<String> set = new HashSet<>();
        for (HardwareItem hardwareItem2 : hardwareItemMap.getAllHardwareItems()) {
            List<HardwareItem> hardwareItemsForWebcam2 = hardwareItemsForWebcam;
            String identifierForFtcJava = HardwareItem.makeIdentifier(hardwareItem2.deviceName);
            List<String> resourceNames2 = resourceNames;
            if (RESERVED_WORDS_FOR_FTCJAVA.contains(identifierForFtcJava)) {
                identifierForFtcJava = identifierForFtcJava + hardwareItem2.hardwareType.identifierSuffixForFtcJava;
            }
            if (!set.add(identifierForFtcJava)) {
                identifierForFtcJava = identifierForFtcJava + hardwareItem2.hardwareType.identifierSuffixForFtcJava;
                set.add(identifierForFtcJava);
            }
            jsHardware.append("    case '").append(hardwareItem2.identifier).append("':\n").append("      return '").append(identifierForFtcJava).append("';\n");
            hardwareItemsForWebcam = hardwareItemsForWebcam2;
            resourceNames = resourceNames2;
            identifiersForJavaScript = identifiersForJavaScript;
        }
        jsHardware.append("  }\n").append("  throw 'Unexpected identifier (' + identifier + ').';\n").append("}\n\n");
        jsHardware.append("function addReservedWordsForFtcJava() {\n");
        for (String word : RESERVED_WORDS_FOR_FTCJAVA) {
            jsHardware.append("  Blockly.FtcJava.addReservedWords('").append(word).append("');\n");
        }
        Set<String> additionalReservedWordsForFtcJava3 = additionalReservedWordsForFtcJava2;
        for (String word2 : additionalReservedWordsForFtcJava3) {
            jsHardware.append("  Blockly.FtcJava.addReservedWords('").append(word2).append("');\n");
        }
        Iterator<HardwareItem> it5 = hardwareItemMap.getAllHardwareItems().iterator();
        while (it5.hasNext()) {
            jsHardware.append("  Blockly.FtcJava.addReservedWords(getHardwareItemIdentifierForFtcJava('").append(it5.next().identifier).append("'));\n");
        }
        Identifier[] identifierArrValues2 = Identifier.values();
        int length3 = identifierArrValues2.length;
        int i6 = 0;
        while (i6 < length3) {
            Identifier identifier3 = identifierArrValues2[i6];
            Identifier[] identifierArr = identifierArrValues2;
            if (identifier3.identifierForFtcJava == null || identifier3.identifierForFtcJava.isEmpty()) {
                i = length3;
            } else {
                i = length3;
                jsHardware.append("  Blockly.FtcJava.addReservedWords('").append(identifier3.identifierForFtcJava).append("');\n");
            }
            i6++;
            identifierArrValues2 = identifierArr;
            length3 = i;
        }
        jsHardware.append("}\n\n");
        jsHardware.append("function knownTypeToClassName(type) {\n").append("  switch (type) {\n");
        Iterator<Map.Entry<String, Class<?>[]>> it6 = KNOWN_TYPES_FOR_FTCJAVA.entrySet().iterator();
        while (it6.hasNext()) {
            Map.Entry<String, Class<?>[]> entry3 = it6.next();
            String pkg = entry3.getKey();
            Class<?>[] value = entry3.getValue();
            int length4 = value.length;
            Iterator<Map.Entry<String, Class<?>[]>> it7 = it6;
            int i7 = 0;
            while (i7 < length4) {
                Map.Entry<String, Class<?>[]> entry4 = entry3;
                Class<?> c = value[i7];
                Set<String> additionalReservedWordsForFtcJava4 = additionalReservedWordsForFtcJava3;
                if (pkg.equals(c.getPackage().getName())) {
                    clsArr = value;
                } else {
                    clsArr = value;
                    RobotLog.e("Error: expected package " + pkg + " for " + c);
                }
                String name = c.getSimpleName();
                Class<?> enclosingClass = c.getEnclosingClass();
                while (enclosingClass != null) {
                    name = enclosingClass.getSimpleName() + DescriptorUtils.JAVA_PACKAGE_SEPARATOR + name;
                    enclosingClass = enclosingClass.getEnclosingClass();
                    skyStoneSoundResourceTooltips = skyStoneSoundResourceTooltips;
                }
                StringBuilder skyStoneSoundResourceTooltips2 = skyStoneSoundResourceTooltips;
                jsHardware.append("    case '").append(name).append("':\n");
                if (c.equals(CRServo.class) || c.equals(DcMotor.class)) {
                    jsHardware.append("    case '").append(name).append(".Direction':\n");
                }
                i7++;
                entry3 = entry4;
                value = clsArr;
                additionalReservedWordsForFtcJava3 = additionalReservedWordsForFtcJava4;
                skyStoneSoundResourceTooltips = skyStoneSoundResourceTooltips2;
            }
            jsHardware.append("      return '").append(pkg).append(".' + type;\n");
            it6 = it7;
        }
        jsHardware.append("    case '").append(Point.class.getName()).append("':\n").append("    case '").append(Rect.class.getName()).append("':\n").append("    case '").append(Size.class.getName()).append("':\n").append("      return type;\n");
        jsHardware.append("  }\n").append("  return knownTypeToClassNameObsolete(type);\n").append("}\n\n");
        jsHardware.append("function getIconClass(categoryName) {\n");
        for (HardwareType hardwareType3 : HardwareType.values()) {
            if (hardwareType3.toolboxCategoryName != null && hardwareType3.toolboxIcon != null) {
                jsHardware.append("  if (categoryName == '").append(ProjectsUtil.escapeSingleQuotes(hardwareType3.toolboxCategoryName)).append("') {\n").append("    return '").append(ProjectsUtil.escapeSingleQuotes(hardwareType3.toolboxIcon.cssClass)).append("';\n").append("  }\n");
            }
        }
        jsHardware.append("  if (categoryName == '").append(ProjectsUtil.escapeSingleQuotes(DC_MOTOR_DUAL_CATEGORY_NAME)).append("') {\n").append("    return '").append(ProjectsUtil.escapeSingleQuotes(ToolboxIcon.DC_MOTOR.cssClass)).append("';\n").append("  }\n").append("  if (categoryName == '").append(ProjectsUtil.escapeSingleQuotes(GAMEPAD_CATEGORY_NAME)).append("') {\n").append("    return '").append(ProjectsUtil.escapeSingleQuotes(ToolboxIcon.GAMEPAD.cssClass)).append("';\n").append("  }\n").append("  if (categoryName == '").append(ProjectsUtil.escapeSingleQuotes(LINEAR_OP_MODE_CATEGORY_NAME)).append("') {\n").append("    return '").append(ProjectsUtil.escapeSingleQuotes(ToolboxIcon.LINEAR_OPMODE.cssClass)).append("';\n").append("  }\n").append("  if (categoryName == '").append(ProjectsUtil.escapeSingleQuotes(COLOR_CATEGORY_NAME)).append("') {\n").append("    return '").append(ProjectsUtil.escapeSingleQuotes(ToolboxIcon.COLOR_SENSOR.cssClass)).append("';\n").append("  }\n").append("  if (categoryName == '").append(ProjectsUtil.escapeSingleQuotes(ELAPSED_TIME_CATEGORY_NAME)).append("') {\n").append("    return '").append(ProjectsUtil.escapeSingleQuotes(ToolboxIcon.ELAPSED_TIME.cssClass)).append("';\n").append("  }\n").append(str5).append("}\n\n");
        jsHardware.append("function getWarningForCapabilityRequestedBySample(capability) {\n").append("  switch (capability) {\n");
        Capability[] capabilityArrValues = Capability.values();
        int length5 = capabilityArrValues.length;
        int i8 = 0;
        while (i8 < length5) {
            Capability capability = capabilityArrValues[i8];
            Map<Capability, Boolean> capabilities3 = capabilities;
            if (!capabilities3.get(capability).booleanValue() && (warning = getCapabilityWarning(capability)) != null) {
                jsHardware.append("    case '").append(capability).append("':\n").append("      return '").append(warning).append("';\n");
            }
            i8++;
            capabilities = capabilities3;
        }
        jsHardware.append("  }\n").append(str5).append("}\n\n");
        jsHardware.append("function getToolbox() {\n").append("  return '").append(ProjectsUtil.escapeSingleQuotes(toolbox2)).append("';\n").append("}\n\n");
        return jsHardware.toString();
    }

    static String makeVisibleNameForDropdownItem(String name) {
        int length = name.length();
        StringBuilder visibleName = new StringBuilder();
        for (int i = 0; i < length; i++) {
            char ch = name.charAt(i);
            if (ch == ' ') {
                visibleName.append((char) 160);
            } else {
                visibleName.append(ch);
            }
        }
        return visibleName.toString();
    }

    private static void addLanguage(String languageCode, String languageName, StringBuilder dropdown, StringBuilder tooltips) {
        dropdown.append("      ['").append(ProjectsUtil.escapeSingleQuotes(makeVisibleNameForDropdownItem(languageCode))).append("', '").append(ProjectsUtil.escapeSingleQuotes(languageCode)).append("'],\n");
        tooltips.append("  ['").append(ProjectsUtil.escapeSingleQuotes(languageCode)).append("', 'The language code for ").append(ProjectsUtil.escapeSingleQuotes(languageName)).append(".'],\n");
    }

    private static void addCountry(String countryCode, String countryName, StringBuilder dropdown, StringBuilder tooltips) {
        dropdown.append("      ['").append(ProjectsUtil.escapeSingleQuotes(makeVisibleNameForDropdownItem(countryCode))).append("', '").append(ProjectsUtil.escapeSingleQuotes(countryCode)).append("'],\n");
        tooltips.append("  ['").append(ProjectsUtil.escapeSingleQuotes(countryCode)).append("', 'The country code for ").append(ProjectsUtil.escapeSingleQuotes(countryName)).append(".'],\n");
    }

    private static void appendCreateDropdownFunction(StringBuilder jsHardware, String functionName, List<HardwareItem> hardwareItems) {
        jsHardware.append("function ").append(functionName).append("() {\n").append("  var CHOICES = [\n");
        for (HardwareItem hardwareItem : hardwareItems) {
            jsHardware.append("      ['").append(ProjectsUtil.escapeSingleQuotes(hardwareItem.visibleName)).append("', '").append(hardwareItem.identifier).append("'],\n");
        }
        jsHardware.append("  ];\n").append("  return createFieldDropdown(CHOICES);\n").append("}\n\n");
    }

    private static List<HardwareItem> getHardwareItemsForDcMotorEx(List<HardwareItem> hardwareItemsForDcMotor) {
        List<HardwareItem> hardwareItemsForDcMotorEx = new ArrayList<>();
        for (HardwareItem hardwareItemForDcMotor : hardwareItemsForDcMotor) {
            hardwareItemsForDcMotorEx.add(hardwareItemForDcMotor);
        }
        return hardwareItemsForDcMotorEx;
    }

    private static String generateToolbox(HardwareItemMap hardwareItemMap, Map<Capability, Boolean> capabilities, AssetManager assetManager, Set<String> additionalReservedWordsForFtcJava, Set<String> methodLookupStrings, StringBuilder jsHardware) throws Throwable {
        StringBuilder xmlToolbox = new StringBuilder();
        xmlToolbox.append("<xml id=\"toolbox\" style=\"display: none\">\n");
        if (assetManager != null) {
            addAsset(xmlToolbox, assetManager, "toolbox/linear_op_mode.xml");
            addAsset(xmlToolbox, assetManager, "toolbox/gamepad.xml");
        }
        for (ToolboxFolder toolboxFolder : ToolboxFolder.values()) {
            xmlToolbox.append(" <category name=\"").append(toolboxFolder.label).append("\">\n");
            SortedSet<HardwareType> hardwareTypes = new TreeSet<>(HardwareType.BY_TOOLBOX_CATEGORY_NAME);
            hardwareTypes.addAll(hardwareItemMap.getHardwareTypes());
            for (HardwareType hardwareType : hardwareTypes) {
                if (hardwareType.toolboxFolder == toolboxFolder && hardwareType.toolboxCategoryName != null) {
                    addHardwareCategoryToToolbox(xmlToolbox, hardwareType, hardwareItemMap.getHardwareItems(hardwareType), assetManager);
                }
            }
            xmlToolbox.append(" </category>\n");
        }
        addAndroidCategoriesToToolbox(xmlToolbox, assetManager);
        addExportedHardwareAndEnums(xmlToolbox, additionalReservedWordsForFtcJava, methodLookupStrings, jsHardware);
        addExportedStaticMethodsAndEnums(xmlToolbox, additionalReservedWordsForFtcJava, methodLookupStrings);
        if (assetManager != null) {
            addAssetWithPlaceholders(xmlToolbox, assetManager, additionalReservedWordsForFtcJava, methodLookupStrings, capabilities, hardwareItemMap, "toolbox/utilities.xml");
            addAsset(xmlToolbox, assetManager, "toolbox/misc.xml");
        }
        xmlToolbox.append(ProjectsUtil.XML_END_TAG);
        return xmlToolbox.toString();
    }

    private static void addExportedHardwareAndEnums(StringBuilder xmlToolbox, Set<String> additionalReservedWordsForFtcJava, Set<String> methodLookupStrings, StringBuilder jsHardware) {
        Map<Class<? extends HardwareDevice>, Set<Method>> methodsByClass = blocksClassFilter.getHardwareMethodsByClass();
        if (methodsByClass.isEmpty()) {
            return;
        }
        OpModeManagerImpl opModeManagerImpl = OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().getRootActivity());
        if (opModeManagerImpl == null) {
            RobotLog.w("Fetching blocks toolbox: Unable to get OpModeManagerImpl");
            return;
        }
        long startTime = System.nanoTime();
        while (opModeManagerImpl.getRobotState() != RobotState.RUNNING) {
            if (System.nanoTime() - startTime < 60000000000L) {
                try {
                    Thread.sleep(1000L);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            } else {
                return;
            }
        }
        HardwareMap hardwareMap = OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().getRootActivity()).getHardwareMap();
        if (hardwareMap == null) {
            RobotLog.w("Fetching blocks toolbox: Unable to get HardwareMap");
            return;
        }
        boolean addedAdditionalHardwareCategory = false;
        Map<Class, Set<Class<? extends Enum>>> enumClassesByEnclosingClass = blocksClassFilter.getEnumClassesByEnclosingClass();
        Iterator<Map.Entry<Class<? extends HardwareDevice>, Set<Method>>> it = methodsByClass.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<Class<? extends HardwareDevice>, Set<Method>> entry = it.next();
            Class<? extends HardwareDevice> clazz = entry.getKey();
            SortedSet<String> deviceNames = hardwareMap.getAllNames(clazz);
            if (!deviceNames.isEmpty()) {
                if (!addedAdditionalHardwareCategory) {
                    xmlToolbox.append("<category name=\"Additional Hardware\">\n");
                    addedAdditionalHardwareCategory = true;
                }
                DeviceProperties deviceProperties = (DeviceProperties) clazz.getAnnotation(DeviceProperties.class);
                HardwareMap hardwareMap2 = hardwareMap;
                String createDropdownFunctionName = processDeviceNames(jsHardware, deviceProperties, deviceNames);
                Map<Class<? extends HardwareDevice>, Set<Method>> methodsByClass2 = methodsByClass;
                String fullClassName = clazz.getName();
                getUserVisibleClassName(fullClassName, additionalReservedWordsForFtcJava);
                OpModeManagerImpl opModeManagerImpl2 = opModeManagerImpl;
                long startTime2 = startTime;
                String str = "\">\n";
                xmlToolbox.append("<category name=\"").append(deviceProperties.name()).append("\">\n");
                Set<Method> methods = entry.getValue();
                Iterator<Method> it2 = methods.iterator();
                while (it2.hasNext()) {
                    Set<Method> methods2 = methods;
                    Method method = it2.next();
                    Iterator<Method> it3 = it2;
                    ExportToBlocks exportToBlocks = (ExportToBlocks) method.getAnnotation(ExportToBlocks.class);
                    if (exportToBlocks == null) {
                        it2 = it3;
                        methods = methods2;
                    } else {
                        boolean addedAdditionalHardwareCategory2 = addedAdditionalHardwareCategory;
                        String returnType = method.getReturnType().getName();
                        Iterator<Map.Entry<Class<? extends HardwareDevice>, Set<Method>>> it4 = it;
                        String blockType = returnType.equals("void") ? "misc_callHardware_noReturn" : "misc_callHardware_return";
                        Map.Entry<Class<? extends HardwareDevice>, Set<Method>> entry2 = entry;
                        String methodName = method.getName();
                        SortedSet<String> deviceNames2 = deviceNames;
                        Class<?>[] parameterTypes = method.getParameterTypes();
                        DeviceProperties deviceProperties2 = deviceProperties;
                        int color = exportToBlocks.color();
                        String comment = exportToBlocks.comment();
                        String tooltip = exportToBlocks.tooltip();
                        String methodLookupString = BlocksClassFilter.getLookupString(method);
                        methodLookupStrings.add(methodLookupString);
                        xmlToolbox.append("<block type=\"").append(blockType).append(str).append("<field name=\"METHOD_NAME\">").append(methodName).append("</field>").append("<mutation").append(" createDropdownFunctionName=\"").append(createDropdownFunctionName).append("\"").append(" methodLookupString=\"").append(methodLookupString).append("\"").append(" fullClassName=\"").append(fullClassName).append("\"").append(" simpleName=\"").append(clazz.getSimpleName()).append("\"").append(" parameterCount=\"").append(parameterTypes.length).append("\"").append(" returnType=\"").append(returnType).append("\"").append(" color=\"").append(color).append("\"").append(" heading=\"\"").append(" comment=\"").append(ToolboxUtil.escapeForXml(comment)).append("\"").append(" tooltip=\"").append(ToolboxUtil.escapeForXml(tooltip)).append("\"").append(" accessMethod=\"").append(accessMethod(true, method.getReturnType())).append("\"").append(" convertReturnValue=\"").append(convertReturnValue(method.getReturnType())).append("\"");
                        processMethodArguments(xmlToolbox, parameterTypes, getParameterLabels(method), getParameterDefaultValues(method));
                        it2 = it3;
                        methods = methods2;
                        it = it4;
                        addedAdditionalHardwareCategory = addedAdditionalHardwareCategory2;
                        entry = entry2;
                        deviceNames = deviceNames2;
                        deviceProperties = deviceProperties2;
                        str = str;
                        createDropdownFunctionName = createDropdownFunctionName;
                        fullClassName = fullClassName;
                    }
                }
                boolean addedAdditionalHardwareCategory3 = addedAdditionalHardwareCategory;
                Iterator<Map.Entry<Class<? extends HardwareDevice>, Set<Method>>> it5 = it;
                Set<Class<? extends Enum>> enumClasses = enumClassesByEnclosingClass.get(clazz);
                if (enumClasses != null) {
                    for (Class<? extends Enum> enumClass : enumClasses) {
                        addExportedEnum(enumClass, xmlToolbox, additionalReservedWordsForFtcJava);
                    }
                }
                xmlToolbox.append("</category>\n");
                hardwareMap = hardwareMap2;
                methodsByClass = methodsByClass2;
                opModeManagerImpl = opModeManagerImpl2;
                startTime = startTime2;
                it = it5;
                addedAdditionalHardwareCategory = addedAdditionalHardwareCategory3;
            }
        }
        if (addedAdditionalHardwareCategory) {
            xmlToolbox.append("</category>\n");
        }
    }

    private static String processDeviceNames(StringBuilder jsHardware, DeviceProperties deviceProperties, SortedSet<String> deviceNames) {
        StringBuilder sb = new StringBuilder();
        String xmlTag = deviceProperties.xmlTag();
        for (int i = 0; i < xmlTag.length(); i++) {
            char ch = xmlTag.charAt(i);
            if (Character.isJavaIdentifierPart(ch)) {
                sb.append(ch);
            } else {
                sb.append('_');
            }
        }
        String functionName = sb.toString();
        jsHardware.append("function ").append(functionName).append("() {\n").append("  var CHOICES = [\n");
        for (String deviceName : deviceNames) {
            String escapedDeviceName = ProjectsUtil.escapeSingleQuotes(deviceName);
            jsHardware.append("      ['").append(escapedDeviceName).append("', '").append(escapedDeviceName).append("'],\n");
        }
        jsHardware.append("  ];\n").append("  return createFieldDropdown(CHOICES);\n").append("}\n\n");
        return functionName;
    }

    private static void addExportedStaticMethodsAndEnums(StringBuilder xmlToolbox, Set<String> additionalReservedWordsForFtcJava, Set<String> methodLookupStrings) {
        boolean addedJavaClassesCategory = false;
        Map<Class, Set<Method>> methodsByClass = blocksClassFilter.getStaticMethodsByClass();
        Map<Class, Set<Class<? extends Enum>>> enumClassesByEnclosingClass = blocksClassFilter.getEnumClassesByEnclosingClass();
        Iterator<Map.Entry<Class, Set<Method>>> it = methodsByClass.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<Class, Set<Method>> entry = it.next();
            if (!addedJavaClassesCategory) {
                xmlToolbox.append("<category name=\"Java Classes\">\n");
                addedJavaClassesCategory = true;
            }
            Class enclosingClass = entry.getKey();
            String fullClassName = enclosingClass.getName();
            String userVisibleClassName = getUserVisibleClassName(fullClassName, additionalReservedWordsForFtcJava);
            xmlToolbox.append("<category name=\"").append(userVisibleClassName).append("\">\n");
            Set<Method> methods = entry.getValue();
            Iterator<Method> it2 = methods.iterator();
            while (it2.hasNext()) {
                Method method = it2.next();
                ExportToBlocks exportToBlocks = (ExportToBlocks) method.getAnnotation(ExportToBlocks.class);
                if (exportToBlocks != null) {
                    boolean addedJavaClassesCategory2 = addedJavaClassesCategory;
                    String returnType = method.getReturnType().getName();
                    Iterator<Map.Entry<Class, Set<Method>>> it3 = it;
                    String blockType = returnType.equals("void") ? "misc_callJava_noReturn" : "misc_callJava_return";
                    Map.Entry<Class, Set<Method>> entry2 = entry;
                    String methodName = method.getName();
                    Set<Method> methods2 = methods;
                    Class<?>[] parameterTypes = method.getParameterTypes();
                    Iterator<Method> it4 = it2;
                    int color = exportToBlocks.color();
                    String heading = exportToBlocks.heading();
                    String comment = exportToBlocks.comment();
                    String tooltip = exportToBlocks.tooltip();
                    String methodLookupString = BlocksClassFilter.getLookupString(method);
                    methodLookupStrings.add(methodLookupString);
                    xmlToolbox.append("<block type=\"").append(blockType).append("\">\n").append("<field name=\"CLASS_NAME\">").append(userVisibleClassName).append("</field>").append("<field name=\"METHOD_NAME\">").append(methodName).append("</field>").append("<mutation").append(" methodLookupString=\"").append(methodLookupString).append("\"").append(" fullClassName=\"").append(fullClassName).append("\"").append(" simpleName=\"").append(enclosingClass.getSimpleName()).append("\"").append(" parameterCount=\"").append(parameterTypes.length).append("\"").append(" returnType=\"").append(returnType).append("\"").append(" color=\"").append(color).append("\"").append(" heading=\"").append(ToolboxUtil.escapeForXml(heading)).append("\"").append(" comment=\"").append(ToolboxUtil.escapeForXml(comment)).append("\"").append(" tooltip=\"").append(ToolboxUtil.escapeForXml(tooltip)).append("\"").append(" accessMethod=\"").append(accessMethod(false, method.getReturnType())).append("\"").append(" convertReturnValue=\"").append(convertReturnValue(method.getReturnType())).append("\"");
                    processMethodArguments(xmlToolbox, parameterTypes, getParameterLabels(method), getParameterDefaultValues(method));
                    it = it3;
                    addedJavaClassesCategory = addedJavaClassesCategory2;
                    entry = entry2;
                    methods = methods2;
                    it2 = it4;
                    methodsByClass = methodsByClass;
                    userVisibleClassName = userVisibleClassName;
                }
            }
            boolean addedJavaClassesCategory3 = addedJavaClassesCategory;
            Map<Class, Set<Method>> methodsByClass2 = methodsByClass;
            Iterator<Map.Entry<Class, Set<Method>>> it5 = it;
            Set<Class<? extends Enum>> enumClasses = enumClassesByEnclosingClass.get(enclosingClass);
            if (enumClasses != null) {
                for (Class<? extends Enum> enumClass : enumClasses) {
                    addExportedEnum(enumClass, xmlToolbox, additionalReservedWordsForFtcJava);
                }
            }
            xmlToolbox.append("</category>\n");
            it = it5;
            addedJavaClassesCategory = addedJavaClassesCategory3;
            methodsByClass = methodsByClass2;
        }
        Set<Class> exportedClasses = methodsByClass.keySet();
        Set<Class<? extends HardwareDevice>> hardwareClasses = blocksClassFilter.getHardwareMethodsByClass().keySet();
        for (Map.Entry<Class, Set<Class<? extends Enum>>> entry3 : enumClassesByEnclosingClass.entrySet()) {
            Class enclosingClass2 = entry3.getKey();
            if (!exportedClasses.contains(enclosingClass2) && !hardwareClasses.contains(enclosingClass2)) {
                if (!addedJavaClassesCategory) {
                    xmlToolbox.append("<category name=\"Java Classes\">\n");
                    addedJavaClassesCategory = true;
                }
                xmlToolbox.append("<category name=\"").append(getUserVisibleClassName(enclosingClass2.getName(), additionalReservedWordsForFtcJava)).append("\">\n");
                for (Class<? extends Enum> enumClass2 : entry3.getValue()) {
                    boolean addedJavaClassesCategory4 = addedJavaClassesCategory;
                    addExportedEnum(enumClass2, xmlToolbox, additionalReservedWordsForFtcJava);
                    addedJavaClassesCategory = addedJavaClassesCategory4;
                }
                xmlToolbox.append("</category>\n");
            }
        }
        if (addedJavaClassesCategory) {
            xmlToolbox.append("</category>\n");
        }
    }

    private static void addExportedEnum(Class<? extends Enum> enumClass, StringBuilder xmlToolbox, Set<String> additionalReservedWordsForFtcJava) {
        ExportEnumToBlocks exportEnumToBlocks = (ExportEnumToBlocks) enumClass.getAnnotation(ExportEnumToBlocks.class);
        if (exportEnumToBlocks == null) {
            return;
        }
        String fullClassName = enumClass.getName();
        String userVisibleEnumName = getUserVisibleEnumName(fullClassName, additionalReservedWordsForFtcJava);
        int color = exportEnumToBlocks.color();
        Object[] enumValues = enumClass.getEnumConstants();
        StringBuilder sb = new StringBuilder().append("<block type=\"").append("misc_enumJava").append("\">\n").append("<field name=\"ENUM_NAME\">").append(userVisibleEnumName).append("</field>").append("<mutation").append(" fullClassName=\"").append(fullClassName).append("\"").append(" color=\"").append(color).append("\"").append(" enumValueCount=\"").append(enumValues.length).append("\"");
        for (int i = 0; i < enumValues.length; i++) {
            sb.append(" enumValue").append(i).append("=\"").append(enumValues[i].toString()).append("\"");
        }
        sb.append("/>");
        String blockDefinitionStart = sb.toString();
        for (Object enumValue : enumValues) {
            xmlToolbox.append(blockDefinitionStart).append("<field name=\"ENUM_VALUE\">").append(enumValue).append("</field>\n").append("</block>\n");
            if (enumValues.length > 4) {
                return;
            }
        }
    }

    private static String getUserVisibleClassName(String fullClassName, Set<String> additionalReservedWordsForFtcJava) {
        String className = fullClassName;
        if (className.startsWith("org.firstinspires.ftc.teamcode.") && className.lastIndexOf(46) == 30) {
            className = className.substring(31);
            additionalReservedWordsForFtcJava.add(className);
        }
        return className.replace('$', DescriptorUtils.JAVA_PACKAGE_SEPARATOR);
    }

    private static String getUserVisibleEnumName(String fullClassName, Set<String> additionalReservedWordsForFtcJava) {
        int firstDollar;
        String className = fullClassName;
        int lastDot = className.lastIndexOf(46);
        if (lastDot != -1 && (firstDollar = (className = className.substring(lastDot + 1)).indexOf(36)) != -1) {
            String outerClassName = className.substring(0, firstDollar);
            additionalReservedWordsForFtcJava.add(outerClassName);
        }
        return className.replace('$', DescriptorUtils.JAVA_PACKAGE_SEPARATOR);
    }

    /* JADX WARN: Removed duplicated region for block: B:387:0x0954  */
    /* JADX WARN: Removed duplicated region for block: B:403:0x097a A[SYNTHETIC] */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    private static void processMethodArguments(java.lang.StringBuilder r20, java.lang.Class[] r21, java.lang.String[] r22, java.lang.String[] r23) {
        /*
            Method dump skipped, instruction units count: 2456
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.google.blocks.ftcrobotcontroller.hardware.HardwareUtil.processMethodArguments(java.lang.StringBuilder, java.lang.Class[], java.lang.String[], java.lang.String[]):void");
    }

    private static <T extends Enum<T>> String parseEnumDefaultValue(String s, Class<T> enumClass) {
        try {
            Enum.valueOf(enumClass, s);
            return s;
        } catch (IllegalArgumentException e) {
            String s2 = s.toUpperCase(Locale.ENGLISH);
            try {
                Enum.valueOf(enumClass, s2);
                return s2;
            } catch (IllegalArgumentException e2) {
                return null;
            }
        }
    }

    public static String[] getParameterLabels(Method method) {
        String[] parameterLabels;
        ExportToBlocks exportToBlocks = (ExportToBlocks) method.getAnnotation(ExportToBlocks.class);
        if (exportToBlocks != null) {
            parameterLabels = exportToBlocks.parameterLabels();
        } else {
            parameterLabels = new String[0];
        }
        int length = method.getParameterTypes().length;
        if (parameterLabels.length != length) {
            parameterLabels = new String[length];
            for (int i = 0; i < parameterLabels.length; i++) {
                parameterLabels[i] = "";
            }
        }
        return parameterLabels;
    }

    public static String[] getParameterDefaultValues(Method method) {
        String[] parameterDefaultValues;
        ExportToBlocks exportToBlocks = (ExportToBlocks) method.getAnnotation(ExportToBlocks.class);
        if (exportToBlocks != null) {
            parameterDefaultValues = exportToBlocks.parameterDefaultValues();
        } else {
            parameterDefaultValues = new String[0];
        }
        int length = method.getParameterTypes().length;
        if (parameterDefaultValues.length != length) {
            parameterDefaultValues = new String[length];
            for (int i = 0; i < parameterDefaultValues.length; i++) {
                parameterDefaultValues[i] = "";
            }
        }
        return parameterDefaultValues;
    }

    private static String accessMethod(boolean hardware, Class returnType) {
        return (returnType.equals(Boolean.TYPE) || returnType.equals(Boolean.class)) ? hardware ? "callHardware_boolean" : "callJava_boolean" : (returnType.equals(Character.TYPE) || returnType.equals(Character.class) || returnType.equals(String.class) || returnType.equals(Byte.TYPE) || returnType.equals(Byte.class) || returnType.equals(Short.TYPE) || returnType.equals(Short.class) || returnType.equals(Integer.TYPE) || returnType.equals(Integer.class) || returnType.equals(Long.TYPE) || returnType.equals(Long.class) || returnType.equals(Float.TYPE) || returnType.equals(Float.class) || returnType.equals(Double.TYPE) || returnType.equals(Double.class) || returnType.isEnum()) ? hardware ? "callHardware_String" : "callJava_String" : hardware ? "callHardware" : "callJava";
    }

    private static String convertReturnValue(Class returnType) {
        if (returnType.equals(Byte.TYPE) || returnType.equals(Byte.class) || returnType.equals(Short.TYPE) || returnType.equals(Short.class) || returnType.equals(Integer.TYPE) || returnType.equals(Integer.class) || returnType.equals(Long.TYPE) || returnType.equals(Long.class) || returnType.equals(Float.TYPE) || returnType.equals(Float.class) || returnType.equals(Double.TYPE) || returnType.equals(Double.class)) {
            return "Number";
        }
        return "";
    }

    private static String parameterProvidedAutomatically(Class parameterType, String parameterLabel, List<String> gamepads) {
        if (parameterType.equals(LinearOpMode.class) || parameterType.equals(OpMode.class)) {
            return "this";
        }
        if (parameterType.equals(HardwareMap.class)) {
            return "hardwareMap";
        }
        if (parameterType.equals(Telemetry.class)) {
            return "telemetry";
        }
        if (parameterType.equals(Gamepad.class)) {
            if (parameterLabel.equals("gamepad1") || parameterLabel.equals("gamepad2")) {
                return parameterLabel;
            }
            if (gamepads.isEmpty()) {
                gamepads.add("gamepad1");
                gamepads.add("gamepad2");
            }
            return gamepads.remove(0);
        }
        return null;
    }

    private static String getCapabilityWarning(Capability capability) {
        switch (capability) {
            case SWITCHABLE_CAMERA:
                return "The current configuration does not have multiple webcams.";
            case VISION:
                return "The current configuration has no webcam.";
            default:
                return null;
        }
    }

    public static Map<Capability, Boolean> getCapabilities(HardwareItemMap hardwareItemMap) {
        boolean hasBuiltinCamera;
        Map<Capability, Boolean> capabilities = new HashMap<>();
        if (Device.isRevControlHub()) {
            hasBuiltinCamera = false;
        } else {
            hasBuiltinCamera = AppUtil.getDefContext().getPackageManager().hasSystemFeature("android.hardware.camera");
        }
        int numberOfWebcams = hardwareItemMap.getHardwareItems(HardwareType.WEBCAM_NAME).size();
        boolean webcam = numberOfWebcams > 0;
        boolean switchableCamera = numberOfWebcams > 1;
        capabilities.put(Capability.BUILTIN_CAMERA, Boolean.valueOf(hasBuiltinCamera));
        capabilities.put(Capability.WEBCAM, Boolean.valueOf(webcam));
        capabilities.put(Capability.SWITCHABLE_CAMERA, Boolean.valueOf(switchableCamera));
        capabilities.put(Capability.VISION, Boolean.valueOf(hasBuiltinCamera || webcam));
        return capabilities;
    }

    private static void addAsset(StringBuilder sb, AssetManager assetManager, String assetName) throws IOException {
        FileUtil.readAsset(sb, assetManager, assetName);
    }

    private static void addAssetSansXmlComments(StringBuilder xmlToolbox, AssetManager assetManager, String assetName) throws IOException {
        BufferedReader reader = new BufferedReader(new InputStreamReader(assetManager.open(assetName)));
        while (true) {
            try {
                String line = reader.readLine();
                if (line != null) {
                    String trimmedLine = line.trim();
                    if (!trimmedLine.startsWith("<!--") || !trimmedLine.endsWith("-->")) {
                        xmlToolbox.append(line).append("\n");
                    }
                } else {
                    reader.close();
                    return;
                }
            } catch (Throwable th) {
                try {
                    reader.close();
                } catch (Throwable th2) {
                    th.addSuppressed(th2);
                }
                throw th;
            }
        }
    }

    private static void addAssetWithPlaceholders(StringBuilder xmlToolbox, AssetManager assetManager, Set<String> additionalReservedWordsForFtcJava, Set<String> methodLookupStrings, Map<Capability, Boolean> capabilities, HardwareItemMap hardwareItemMap, String assetName) throws Throwable {
        BufferedReader reader = new BufferedReader(new InputStreamReader(assetManager.open(assetName)));
        while (true) {
            try {
                String line = reader.readLine();
                if (line != null) {
                    String line2 = line.trim().replace("<placeholder_webcam_webcamNames/>", getWebcamBlocks(hardwareItemMap)).replace("<placeholder_apriltag_exportedAprilTagLibraries/>", getExportedAprilTagLibraryBlocks(additionalReservedWordsForFtcJava, methodLookupStrings));
                    if (line2.startsWith("<placeholder_") && line2.endsWith("/>")) {
                        int startOfType = "<placeholder_".length();
                        int endOfType = line2.indexOf(95, startOfType);
                        if (endOfType != -1) {
                            String type = line2.substring(startOfType, endOfType);
                            String childAssetName = "toolbox/" + line2.substring(endOfType + 1, line2.length() - "/>".length()).trim() + RobotConfigFileManager.FILE_EXT;
                            Boolean allowed = capabilities.get(Capability.fromPlaceholderType(type));
                            if (allowed == null) {
                                RobotLog.e("Error: Skipping " + childAssetName + " because capability \"" + type + "\" is not recognized.");
                            } else if (!allowed.booleanValue()) {
                                RobotLog.w("Skipping " + childAssetName + " because capability \"" + type + "\" is not supported by this device and/or hardware.");
                            } else {
                                addAssetWithPlaceholders(xmlToolbox, assetManager, additionalReservedWordsForFtcJava, methodLookupStrings, capabilities, hardwareItemMap, childAssetName);
                            }
                        } else {
                            RobotLog.e("Error: Unable to parse placeholder \"" + line2 + "\"");
                        }
                    } else {
                        try {
                            xmlToolbox.append(line2).append("\n");
                        } catch (Throwable th) {
                            th = th;
                            Throwable th2 = th;
                            try {
                                reader.close();
                                throw th2;
                            } catch (Throwable th3) {
                                th2.addSuppressed(th3);
                                throw th2;
                            }
                        }
                    }
                } else {
                    reader.close();
                    return;
                }
            } catch (Throwable th4) {
                th = th4;
            }
        }
    }

    private static String getWebcamBlocks(HardwareItemMap hardwareItemMap) {
        StringBuilder webcamBlocks = new StringBuilder();
        List<HardwareItem> hardwareItemsForWebcam = hardwareItemMap.getHardwareItems(HardwareType.WEBCAM_NAME);
        for (HardwareItem hardwareItemForWebcam : hardwareItemsForWebcam) {
            webcamBlocks.append("<block type=\"navigation_webcamName\"><field name=\"WEBCAM_NAME\">").append(hardwareItemForWebcam.deviceName).append("</field></block>\n");
        }
        return webcamBlocks.toString();
    }

    private static String getExportedAprilTagLibraryBlocks(Set<String> additionalReservedWordsForFtcJava, Set<String> methodLookupStrings) {
        StringBuilder sb = new StringBuilder();
        Map<Class, Set<Method>> methodsByClass = blocksClassFilter.getAprilTagLibraryMethodsByClass();
        if (!methodsByClass.isEmpty()) {
            Iterator<Map.Entry<Class, Set<Method>>> it = methodsByClass.entrySet().iterator();
            while (it.hasNext()) {
                Map.Entry<Class, Set<Method>> entry = it.next();
                Class enclosingClass = entry.getKey();
                String fullClassName = enclosingClass.getName();
                String userVisibleClassName = getUserVisibleClassName(fullClassName, additionalReservedWordsForFtcJava);
                Set<Method> methods = entry.getValue();
                for (Method method : methods) {
                    ExportAprilTagLibraryToBlocks exportAprilTagLibraryToBlocks = (ExportAprilTagLibraryToBlocks) method.getAnnotation(ExportAprilTagLibraryToBlocks.class);
                    if (exportAprilTagLibraryToBlocks != null) {
                        String returnType = method.getReturnType().getName();
                        String blockType = returnType.equals("void") ? "misc_callJava_noReturn" : "misc_callJava_return";
                        String methodName = method.getName();
                        Class<?>[] parameterTypes = method.getParameterTypes();
                        Map<Class, Set<Method>> methodsByClass2 = methodsByClass;
                        int color = exportAprilTagLibraryToBlocks.color();
                        String heading = exportAprilTagLibraryToBlocks.heading();
                        String comment = exportAprilTagLibraryToBlocks.comment();
                        String tooltip = exportAprilTagLibraryToBlocks.tooltip();
                        Iterator<Map.Entry<Class, Set<Method>>> it2 = it;
                        String methodLookupString = BlocksClassFilter.getLookupString(method);
                        methodLookupStrings.add(methodLookupString);
                        sb.append("<block type=\"variables_set\"><field name=\"VAR\">myAprilTagLibrary</field><value name=\"VALUE\">\n").append("<block type=\"").append(blockType).append("\">\n").append("<field name=\"CLASS_NAME\">").append(userVisibleClassName).append("</field>").append("<field name=\"METHOD_NAME\">").append(methodName).append("</field>").append("<mutation").append(" methodLookupString=\"").append(methodLookupString).append("\"").append(" fullClassName=\"").append(fullClassName).append("\"").append(" simpleName=\"").append(enclosingClass.getSimpleName()).append("\"").append(" parameterCount=\"").append(parameterTypes.length).append("\"").append(" returnType=\"").append(returnType).append("\"").append(" color=\"").append(color).append("\"").append(" heading=\"").append(ToolboxUtil.escapeForXml(heading)).append("\"").append(" comment=\"").append(ToolboxUtil.escapeForXml(comment)).append("\"").append(" tooltip=\"").append(ToolboxUtil.escapeForXml(tooltip)).append("\"").append(" accessMethod=\"").append(accessMethod(false, method.getReturnType())).append("\"").append(" convertReturnValue=\"").append(convertReturnValue(method.getReturnType())).append("\"");
                        processMethodArguments(sb, parameterTypes, getParameterLabels(method), getParameterDefaultValues(method));
                        sb.append("</value></block>");
                        methodsByClass = methodsByClass2;
                        it = it2;
                        entry = entry;
                        userVisibleClassName = userVisibleClassName;
                    }
                }
            }
        }
        return sb.toString();
    }

    private static void addAndroidCategoriesToToolbox(StringBuilder xmlToolbox, AssetManager assetManager) throws IOException {
        boolean hasAccelerometer = !sensorManager.getSensorList(1).isEmpty();
        boolean hasGyroscope = !sensorManager.getSensorList(4).isEmpty();
        boolean hasMagneticField = true ^ sensorManager.getSensorList(2).isEmpty();
        StringBuilder sb = new StringBuilder();
        boolean empty = true;
        if (hasAccelerometer) {
            if (assetManager != null) {
                addAsset(sb, assetManager, "toolbox/android_accelerometer.xml");
                empty = false;
            }
        } else {
            RobotLog.w("Skipping toolbox/android_accelerometer.xml because this device does not have an accelerometer.");
        }
        if (hasGyroscope) {
            if (assetManager != null) {
                addAsset(sb, assetManager, "toolbox/android_gyroscope.xml");
                empty = false;
            }
        } else {
            RobotLog.w("Skipping toolbox/android_gyroscope.xml because this device does not have a gyroscope.");
        }
        if (hasAccelerometer && hasMagneticField) {
            if (assetManager != null) {
                addAsset(sb, assetManager, "toolbox/android_orientation.xml");
                empty = false;
            }
        } else {
            RobotLog.w("Skipping toolbox/android_gyroscope.xml because this device does not have an accelerometer and a magnetic field sensor.");
        }
        if (assetManager != null) {
            addAsset(sb, assetManager, "toolbox/android_sound_pool.xml");
            addAsset(sb, assetManager, "toolbox/android_text_to_speech.xml");
            empty = false;
        }
        if (!empty) {
            xmlToolbox.append("<category name=\"Android\">\n").append((CharSequence) sb).append("</category>\n");
        }
    }

    private static void addHardwareCategoryToToolbox(StringBuilder xmlToolbox, HardwareType hardwareType, List<HardwareItem> hardwareItems, AssetManager assetManager) throws IOException {
        if (hardwareItems != null && hardwareItems.size() > 0) {
            xmlToolbox.append("  <category name=\"").append(hardwareType.toolboxCategoryName).append("\">\n");
            switch (hardwareType) {
                case ACCELERATION_SENSOR:
                    addAccelerationSensorCategoryToToolbox(xmlToolbox, hardwareType, hardwareItems);
                    break;
                case ANALOG_INPUT:
                    addAnalogInputCategoryToToolbox(xmlToolbox, hardwareType, hardwareItems);
                    break;
                case ANDY_MARK_COLOR_SENSOR:
                    addAndyMarkColorSensorCategoryToToolbox(xmlToolbox, hardwareType, hardwareItems);
                    break;
                case BNO055IMU:
                    addBNO055IMUCategoryToToolbox(xmlToolbox, hardwareType, hardwareItems);
                    break;
                case COLOR_RANGE_SENSOR:
                    addColorRangeSensorCategoryToToolbox(xmlToolbox, hardwareType, hardwareItems);
                    break;
                case COLOR_SENSOR:
                    addColorSensorCategoryToToolbox(xmlToolbox, hardwareType, hardwareItems);
                    break;
                case COMPASS_SENSOR:
                    addCompassSensorCategoryToToolbox(xmlToolbox, hardwareType, hardwareItems);
                    break;
                case CR_SERVO:
                    addCRServoCategoryToToolbox(xmlToolbox, hardwareType, hardwareItems);
                    break;
                case DC_MOTOR:
                    addDcMotorCategoryToToolbox(xmlToolbox, hardwareType, hardwareItems);
                    break;
                case DIGITAL_CHANNEL:
                    addDigitalChannelCategoryToToolbox(xmlToolbox, hardwareType, hardwareItems);
                    break;
                case DISTANCE_SENSOR:
                    addDistanceSensorCategoryToToolbox(xmlToolbox, hardwareType, hardwareItems);
                    break;
                case GOBILDA_PINPOINT:
                    addGoBildaPinpointCategoryToToolbox(xmlToolbox, hardwareType, hardwareItems);
                    break;
                case GYRO_SENSOR:
                    addGyroSensorCategoryToToolbox(xmlToolbox, hardwareType, hardwareItems);
                    break;
                case HUSKY_LENS:
                    addHuskyLensCategoryToToolbox(xmlToolbox, hardwareType, hardwareItems, assetManager);
                    break;
                case IMU:
                    addImuCategoryToToolbox(xmlToolbox, hardwareType, hardwareItems, assetManager);
                    break;
                case IR_SEEKER_SENSOR:
                    addIrSeekerSensorCategoryToToolbox(xmlToolbox, hardwareType, hardwareItems);
                    break;
                case LED:
                    addLedCategoryToToolbox(xmlToolbox, hardwareType, hardwareItems);
                    break;
                case LIMELIGHT_3A:
                    addLimelight3ACategoryToToolbox(xmlToolbox, hardwareType, hardwareItems, assetManager);
                    break;
                case LIGHT_SENSOR:
                    addLightSensorCategoryToToolbox(xmlToolbox, hardwareType, hardwareItems);
                    break;
                case MAX_SONAR_I2CXL:
                    addMaxSonarI2CXLCategoryToToolbox(xmlToolbox, hardwareType, hardwareItems);
                    break;
                case MR_I2C_COMPASS_SENSOR:
                    addMrI2cCompassSensorCategoryToToolbox(xmlToolbox, hardwareType, hardwareItems);
                    break;
                case MR_I2C_RANGE_SENSOR:
                    addMrI2cRangeSensorCategoryToToolbox(xmlToolbox, hardwareType, hardwareItems);
                    break;
                case OCTOQUAD:
                    addOctoQuadCategoryToToolbox(xmlToolbox, hardwareType, hardwareItems, assetManager);
                    break;
                case OPTICAL_DISTANCE_SENSOR:
                    addOpticalDistanceSensorCategoryToToolbox(xmlToolbox, hardwareType, hardwareItems);
                    break;
                case REV_BLINKIN_LED_DRIVER:
                    addRevBlinkinLedDriverCategoryToToolbox(xmlToolbox, assetManager);
                    break;
                case SERVO:
                    addServoCategoryToToolbox(xmlToolbox, hardwareType, hardwareItems);
                    break;
                case SERVO_CONTROLLER:
                    addServoControllerCategoryToToolbox(xmlToolbox, hardwareType, hardwareItems);
                    break;
                case SPARKFUN_LED_STICK:
                    addSparkFunLEDStickCategoryToToolbox(xmlToolbox, hardwareType, hardwareItems, assetManager);
                    break;
                case SPARKFUN_OTOS:
                    addSparkFunOTOSCategoryToToolbox(xmlToolbox, hardwareType, hardwareItems, assetManager);
                    break;
                case TOUCH_SENSOR:
                    addTouchSensorCategoryToToolbox(xmlToolbox, hardwareType, hardwareItems);
                    break;
                case ULTRASONIC_SENSOR:
                    addUltrasonicSensorCategoryToToolbox(xmlToolbox, hardwareType, hardwareItems);
                    break;
                case VOLTAGE_SENSOR:
                    addVoltageSensorCategoryToToolbox(xmlToolbox, hardwareType, hardwareItems);
                    break;
                default:
                    throw new IllegalArgumentException("Unexpected hardware type " + hardwareType);
            }
            xmlToolbox.append("  </category>\n");
            if (assetManager != null) {
                switch (hardwareType) {
                    case BNO055IMU:
                        addAsset(xmlToolbox, assetManager, "toolbox/bno055imu_parameters.xml");
                        return;
                    case SPARKFUN_OTOS:
                        addAsset(xmlToolbox, assetManager, "toolbox/sparkfun_otos_pose2d.xml");
                        addAsset(xmlToolbox, assetManager, "toolbox/sparkfun_otos_signalprocessconfig.xml");
                        addAsset(xmlToolbox, assetManager, "toolbox/sparkfun_otos_status.xml");
                        addAsset(xmlToolbox, assetManager, "toolbox/sparkfun_otos_version.xml");
                        return;
                    default:
                        return;
                }
            }
        }
    }

    private static void addAccelerationSensorCategoryToToolbox(StringBuilder xmlToolbox, HardwareType hardwareType, List<HardwareItem> hardwareItems) {
        String identifier = hardwareItems.get(0).identifier;
        SortedMap<String, String> properties = new TreeMap<>();
        properties.put("Acceleration", "Acceleration");
        properties.put("XAccel", "Number");
        properties.put("YAccel", "Number");
        properties.put("ZAccel", "Number");
        ToolboxUtil.addProperties(xmlToolbox, hardwareType, identifier, properties, null, null);
        Map<String, Map<String, String>> functions = new TreeMap<>();
        functions.put("toText", null);
        ToolboxUtil.addFunctions(xmlToolbox, hardwareType, identifier, functions);
    }

    private static void addBNO055IMUCategoryToToolbox(StringBuilder xmlToolbox, HardwareType hardwareType, List<HardwareItem> hardwareItems) {
        String identifier = hardwareItems.get(0).identifier;
        SortedMap<String, String> properties = new TreeMap<>();
        properties.put("Acceleration", "Acceleration");
        properties.put("AngularOrientation", "Orientation");
        properties.put("AngularOrientationAxes", "Array");
        properties.put("AngularVelocity", "AngularVelocity");
        properties.put("AngularVelocityAxes", "Array");
        properties.put("CalibrationStatus", "String");
        properties.put("Gravity", "Acceleration");
        properties.put("I2cAddress7Bit", "Number");
        properties.put("I2cAddress8Bit", "Number");
        properties.put("LinearAcceleration", "Acceleration");
        properties.put("MagneticFieldStrength", "MagneticFlux");
        properties.put("OverallAcceleration", "Acceleration");
        properties.put("Position", "Position");
        properties.put("QuaternionOrientation", "Quaternion");
        properties.put("SystemError", "String");
        properties.put("SystemStatus", "SystemStatus");
        properties.put("Temperature", "Temperature");
        properties.put("Velocity", "Velocity");
        Map<String, String> enumBlocks = new HashMap<>();
        enumBlocks.put("SystemStatus", ToolboxUtil.makeTypedEnumBlock(hardwareType, "systemStatus"));
        ToolboxUtil.addProperties(xmlToolbox, hardwareType, identifier, properties, null, enumBlocks);
        Map<String, Map<String, String>> functions = new TreeMap<>();
        Map<String, String> initializeArgs = new LinkedHashMap<>();
        initializeArgs.put("PARAMETERS", ToolboxUtil.makeVariableGetBlock("parameters"));
        functions.put("initialize", initializeArgs);
        Map<String, String> startAccelerationIntegration_with1Args = new LinkedHashMap<>();
        startAccelerationIntegration_with1Args.put("MS_POLL_INTERVAL", ToolboxUtil.makeNumberShadow(1000));
        functions.put("startAccelerationIntegration_with1", startAccelerationIntegration_with1Args);
        Map<String, String> startAccelerationIntegration_with3Args = new LinkedHashMap<>();
        startAccelerationIntegration_with3Args.put("INITIAL_POSITION", ToolboxUtil.makeVariableGetBlock("position"));
        startAccelerationIntegration_with3Args.put("INITIAL_VELOCITY", ToolboxUtil.makeVariableGetBlock("velocity"));
        startAccelerationIntegration_with3Args.put("MS_POLL_INTERVAL", ToolboxUtil.makeNumberShadow(1000));
        functions.put("startAccelerationIntegration_with3", startAccelerationIntegration_with3Args);
        functions.put("stopAccelerationIntegration", null);
        functions.put("isSystemCalibrated", null);
        functions.put("isGyroCalibrated", null);
        functions.put("isAccelerometerCalibrated", null);
        functions.put("isMagnetometerCalibrated", null);
        Map<String, String> saveCalibrationDataArgs = new LinkedHashMap<>();
        saveCalibrationDataArgs.put("FILE_NAME", ToolboxUtil.makeTextShadow("IMUCalibration.json"));
        functions.put("saveCalibrationData", saveCalibrationDataArgs);
        Map<String, String> getAngularVelocityArgs = new LinkedHashMap<>();
        getAngularVelocityArgs.put("ANGLE_UNIT", ToolboxUtil.makeTypedEnumShadow(NotificationCompat.CATEGORY_NAVIGATION, "angleUnit"));
        functions.put("getAngularVelocity", getAngularVelocityArgs);
        Map<String, String> getAngularOrientationArgs = new LinkedHashMap<>();
        getAngularOrientationArgs.put("AXES_REFERENCE", ToolboxUtil.makeTypedEnumShadow(NotificationCompat.CATEGORY_NAVIGATION, "axesReference"));
        getAngularOrientationArgs.put("AXES_ORDER", ToolboxUtil.makeTypedEnumShadow(NotificationCompat.CATEGORY_NAVIGATION, "axesOrder"));
        getAngularOrientationArgs.put("ANGLE_UNIT", ToolboxUtil.makeTypedEnumShadow(NotificationCompat.CATEGORY_NAVIGATION, "angleUnit"));
        functions.put("getAngularOrientation", getAngularOrientationArgs);
        ToolboxUtil.addFunctions(xmlToolbox, hardwareType, identifier, functions);
    }

    private static void addImuCategoryToToolbox(StringBuilder xmlToolbox, HardwareType hardwareType, List<HardwareItem> hardwareItems, AssetManager assetManager) throws IOException {
        String identifier = hardwareItems.get(0).identifier;
        Map<String, Map<String, String>> initFunctions = new TreeMap<>();
        Map<String, String> initializeArgs = new LinkedHashMap<>();
        initializeArgs.put("PARAMETERS", "<block type=\"imuParameters_create\"></block>");
        initFunctions.put("initialize", initializeArgs);
        ToolboxUtil.addFunctions(xmlToolbox, hardwareType, identifier, initFunctions, null, null, null);
        SortedMap<String, String> properties = new TreeMap<>();
        properties.put("RobotOrientationAsQuaternion", "Quaternion");
        properties.put("RobotYawPitchRollAngles", "YawPitchRollAngles");
        ToolboxUtil.addProperties(xmlToolbox, hardwareType, identifier, properties, null, null);
        Map<String, Map<String, String>> functions = new TreeMap<>();
        Map<String, String> getRobotAngularVelocityArgs = new LinkedHashMap<>();
        getRobotAngularVelocityArgs.put("ANGLE_UNIT", ToolboxUtil.makeTypedEnumShadow(NotificationCompat.CATEGORY_NAVIGATION, "angleUnit"));
        functions.put("getRobotAngularVelocity", getRobotAngularVelocityArgs);
        Map<String, String> getRobotOrientationArgs = new LinkedHashMap<>();
        getRobotOrientationArgs.put("AXES_REFERENCE", ToolboxUtil.makeTypedEnumShadow(NotificationCompat.CATEGORY_NAVIGATION, "axesReference", "AXES_REFERENCE", "INTRINSIC"));
        getRobotOrientationArgs.put("AXES_ORDER", ToolboxUtil.makeTypedEnumShadow(NotificationCompat.CATEGORY_NAVIGATION, "axesOrder", "AXES_ORDER", "ZYX"));
        getRobotOrientationArgs.put("ANGLE_UNIT", ToolboxUtil.makeTypedEnumShadow(NotificationCompat.CATEGORY_NAVIGATION, "angleUnit"));
        functions.put("getRobotOrientation", getRobotOrientationArgs);
        functions.put("resetYaw", null);
        ToolboxUtil.addFunctions(xmlToolbox, hardwareType, identifier, functions);
        if (assetManager != null) {
            addAsset(xmlToolbox, assetManager, "toolbox/imu_orientation.xml");
        }
    }

    private static void addAnalogInputCategoryToToolbox(StringBuilder xmlToolbox, HardwareType hardwareType, List<HardwareItem> hardwareItems) {
        String identifier = hardwareItems.get(0).identifier;
        SortedMap<String, String> properties = new TreeMap<>();
        properties.put("Voltage", "Number");
        properties.put("MaxVoltage", "Number");
        ToolboxUtil.addProperties(xmlToolbox, hardwareType, identifier, properties, null, null);
    }

    private static void addAnalogOutputCategoryToToolbox(StringBuilder xmlToolbox, HardwareType hardwareType, List<HardwareItem> hardwareItems) {
        String identifier = hardwareItems.get(0).identifier;
        Map<String, Map<String, String>> functions = new TreeMap<>();
        Map<String, String> setAnalogOutputVoltageArgs = new LinkedHashMap<>();
        setAnalogOutputVoltageArgs.put("VOLTAGE", ToolboxUtil.makeNumberShadow(512));
        functions.put("setAnalogOutputVoltage_Number", setAnalogOutputVoltageArgs);
        Map<String, String> setAnalogOutputFrequencyArgs = new LinkedHashMap<>();
        setAnalogOutputFrequencyArgs.put("FREQUENCY", ToolboxUtil.makeNumberShadow(100));
        functions.put("setAnalogOutputFrequency_Number", setAnalogOutputFrequencyArgs);
        Map<String, String> setAnalogOutputModeArgs = new LinkedHashMap<>();
        setAnalogOutputModeArgs.put("MODE", ToolboxUtil.makeNumberShadow(0));
        functions.put("setAnalogOutputMode_Number", setAnalogOutputModeArgs);
        ToolboxUtil.addFunctions(xmlToolbox, hardwareType, identifier, functions);
    }

    private static void addAndyMarkColorSensorCategoryToToolbox(StringBuilder xmlToolbox, HardwareType hardwareType, List<HardwareItem> hardwareItems) {
        String identifier = hardwareItems.get(0).identifier;
        String gain4x = ToolboxUtil.makeTypedEnumShadow(hardwareType, "proximityGain", "PROXIMITY_GAIN", "GAIN_4X");
        String six = ToolboxUtil.makeNumberShadow(6);
        String length8us = ToolboxUtil.makeTypedEnumShadow(hardwareType, "proximityPulseLength", "PROXIMITY_PULSE_LENGTH", "LENGTH_8US");
        SortedMap<String, String> properties = new TreeMap<>();
        properties.put("Alpha", "Number");
        properties.put("Argb", "Number");
        properties.put("Red", "Number");
        properties.put("Green", "Number");
        properties.put("Blue", "Number");
        properties.put("LightDetected", "Number");
        properties.put("I2cAddress7Bit", "Number");
        Map<String, String[]> setterValues = new HashMap<>();
        setterValues.put("I2cAddress7Bit", new String[]{ToolboxUtil.makeNumberShadow(57)});
        ToolboxUtil.addProperties(xmlToolbox, hardwareType, identifier, properties, setterValues, null);
        Map<String, Map<String, String>> functions = new TreeMap<>();
        Map<String, String> getDistanceArgs = new LinkedHashMap<>();
        getDistanceArgs.put("UNIT", ToolboxUtil.makeTypedEnumShadow(NotificationCompat.CATEGORY_NAVIGATION, "distanceUnit"));
        functions.put("getDistance", getDistanceArgs);
        functions.put("getNormalizedColors", null);
        Map<String, String> setProximityGainArgs = new LinkedHashMap<>();
        setProximityGainArgs.put("GAIN", gain4x);
        functions.put("setProximityGain", setProximityGainArgs);
        Map<String, String> setProximityLedPulsesArgs = new LinkedHashMap<>();
        setProximityLedPulsesArgs.put("PULSES", six);
        functions.put("setProximityLedPulses", setProximityLedPulsesArgs);
        Map<String, String> setProximityLedPulseLengthArgs = new LinkedHashMap<>();
        setProximityLedPulseLengthArgs.put("PULSE_LENGTH", length8us);
        functions.put("setProximityLedPulseLength", setProximityLedPulseLengthArgs);
        Map<String, String> configureProximitySettingsArgs = new LinkedHashMap<>();
        configureProximitySettingsArgs.put("GAIN", gain4x);
        configureProximitySettingsArgs.put("PULSES", six);
        configureProximitySettingsArgs.put("PULSE_LENGTH", length8us);
        functions.put("configureProximitySettings", configureProximitySettingsArgs);
        ToolboxUtil.addFunctions(xmlToolbox, hardwareType, identifier, functions);
    }

    private static void addColorSensorCategoryToToolbox(StringBuilder xmlToolbox, HardwareType hardwareType, List<HardwareItem> hardwareItems) {
        String identifier = hardwareItems.get(0).identifier;
        SortedMap<String, String> properties = new TreeMap<>();
        properties.put("Red", "Number");
        properties.put("Green", "Number");
        properties.put("Blue", "Number");
        properties.put("Alpha", "Number");
        properties.put("Argb", "Number");
        properties.put("Gain", "Number");
        properties.put("I2cAddress7Bit", "Number");
        properties.put("I2cAddress8Bit", "Number");
        Map<String, String[]> setterValues = new HashMap<>();
        setterValues.put("Gain", new String[]{ToolboxUtil.makeNumberShadow(2)});
        setterValues.put("I2cAddress7Bit", new String[]{ToolboxUtil.makeNumberShadow(8)});
        setterValues.put("I2cAddress8Bit", new String[]{ToolboxUtil.makeNumberShadow(16)});
        ToolboxUtil.addProperties(xmlToolbox, hardwareType, identifier, properties, setterValues, null);
        Map<String, Map<String, String>> functions = new TreeMap<>();
        Map<String, String> enableLedArgs = new LinkedHashMap<>();
        enableLedArgs.put("ENABLE", ToolboxUtil.makeBooleanShadow(true));
        functions.put("enableLed_Boolean", enableLedArgs);
        functions.put("isLightOn", null);
        functions.put("getNormalizedColors", null);
        functions.put("toText", null);
        ToolboxUtil.addFunctions(xmlToolbox, hardwareType, identifier, functions);
    }

    private static void addCompassSensorCategoryToToolbox(StringBuilder xmlToolbox, HardwareType hardwareType, List<HardwareItem> hardwareItems) {
        String identifier = hardwareItems.get(0).identifier;
        SortedMap<String, String> properties = new TreeMap<>();
        properties.put("Direction", "Number");
        properties.put("CalibrationFailed", "Boolean");
        ToolboxUtil.addProperties(xmlToolbox, hardwareType, identifier, properties, null, null);
        Map<String, Map<String, String>> functions = new TreeMap<>();
        Map<String, String> setModeArgs = new LinkedHashMap<>();
        setModeArgs.put("COMPASS_MODE", ToolboxUtil.makeTypedEnumShadow(hardwareType, "compassMode"));
        functions.put("setMode_CompassMode", setModeArgs);
        ToolboxUtil.addFunctions(xmlToolbox, hardwareType, identifier, functions);
    }

    private static void addCRServoCategoryToToolbox(StringBuilder xmlToolbox, HardwareType hardwareType, List<HardwareItem> hardwareItems) {
        String identifier = hardwareItems.get(0).identifier;
        SortedMap<String, String> properties = new TreeMap<>();
        properties.put("Direction", "Direction");
        properties.put("Power", "Number");
        Map<String, String> enumBlocks = new HashMap<>();
        enumBlocks.put("Direction", ToolboxUtil.makeTypedEnumBlock(hardwareType, "direction"));
        Map<String, String[]> setterValues = new HashMap<>();
        setterValues.put("Direction", new String[]{ToolboxUtil.makeTypedEnumShadow(hardwareType, "direction")});
        setterValues.put("Power", new String[]{ToolboxUtil.makeNumberShadow(1), ToolboxUtil.makeNumberShadow(0)});
        ToolboxUtil.addProperties(xmlToolbox, hardwareType, identifier, properties, setterValues, enumBlocks);
        Map<String, Map<String, String>> functions = new TreeMap<>();
        functions.put("setPwmEnable", null);
        functions.put("setPwmDisable", null);
        functions.put("isPwmEnabled", null);
        ToolboxUtil.addFunctions(xmlToolbox, hardwareType, identifier, functions);
    }

    private static void addDcMotorCategoryToToolbox(StringBuilder xmlToolbox, HardwareType hardwareType, List<HardwareItem> hardwareItems) {
        String str;
        Map<String, Map<String, String>> functions;
        String str2;
        String five;
        String identifier;
        String str3;
        SortedMap<String, String> properties;
        String zero;
        String identifier2 = hardwareItems.get(0).identifier;
        String zero2 = ToolboxUtil.makeNumberShadow(0);
        String one = ToolboxUtil.makeNumberShadow(1);
        String five2 = ToolboxUtil.makeNumberShadow(5);
        String ten = ToolboxUtil.makeNumberShadow(10);
        String runMode = ToolboxUtil.makeTypedEnumShadow(hardwareType, "runMode");
        String zeroPowerBehavior = ToolboxUtil.makeTypedEnumShadow(hardwareType, "zeroPowerBehavior");
        String direction = ToolboxUtil.makeTypedEnumShadow(hardwareType, "direction");
        String angleUnit = ToolboxUtil.makeTypedEnumShadow(NotificationCompat.CATEGORY_NAVIGATION, "angleUnit");
        String currentUnit = ToolboxUtil.makeTypedEnumShadow(NotificationCompat.CATEGORY_NAVIGATION, "currentUnit");
        String runModeRunUsingEncoder = ToolboxUtil.makeTypedEnumShadow(hardwareType, "runMode", "RUN_MODE", "RUN_USING_ENCODER");
        SortedMap<String, String> properties2 = new TreeMap<>();
        properties2.put("CurrentPosition", "Number");
        properties2.put("Direction", "Direction");
        properties2.put("Mode", "RunMode");
        properties2.put("Power", "Number");
        properties2.put("PowerFloat", "Boolean");
        properties2.put("TargetPosition", "Number");
        properties2.put("ZeroPowerBehavior", "ZeroPowerBehavior");
        Map<String, String> enumBlocks = new HashMap<>();
        enumBlocks.put("Direction", ToolboxUtil.makeTypedEnumBlock(hardwareType, "direction"));
        enumBlocks.put("Mode", ToolboxUtil.makeTypedEnumBlock(hardwareType, "runMode"));
        enumBlocks.put("ZeroPowerBehavior", ToolboxUtil.makeTypedEnumBlock(hardwareType, "zeroPowerBehavior"));
        Map<String, String[]> setterValues = new HashMap<>();
        setterValues.put("Direction", new String[]{direction});
        setterValues.put("Mode", new String[]{runMode});
        setterValues.put("Power", new String[]{one, zero2});
        setterValues.put("TargetPosition", new String[]{zero2});
        setterValues.put("ZeroPowerBehavior", new String[]{zeroPowerBehavior});
        ToolboxUtil.addProperties(xmlToolbox, hardwareType, identifier2, properties2, setterValues, enumBlocks);
        Map<String, Map<String, String>> functions2 = new TreeMap<>();
        functions2.put("isBusy", null);
        ToolboxUtil.addFunctions(xmlToolbox, hardwareType, identifier2, functions2);
        functions2.clear();
        if (hardwareItems.size() <= 1) {
            str = "    </category>\n";
            functions = functions2;
            str2 = "RUN_MODE";
            five = five2;
            identifier = ten;
            str3 = "    <category name=\"Dual\">\n";
            properties = properties2;
        } else {
            String identifier1 = hardwareItems.get(0).identifier;
            String identifier22 = hardwareItems.get(1).identifier;
            xmlToolbox.append("    <category name=\"Dual\">\n");
            str = "    </category>\n";
            str3 = "    <category name=\"Dual\">\n";
            functions = functions2;
            str2 = "RUN_MODE";
            identifier = ten;
            five = five2;
            properties = properties2;
            ToolboxUtil.addDualPropertySetters(xmlToolbox, hardwareType, "Power", "Number", identifier1, one, identifier22, one);
            ToolboxUtil.addDualPropertySetters(xmlToolbox, hardwareType, "Power", "Number", identifier1, zero2, identifier22, zero2);
            ToolboxUtil.addDualPropertySetters(xmlToolbox, hardwareType, "Mode", "RunMode", identifier1, runMode, identifier22, runMode);
            ToolboxUtil.addDualPropertySetters(xmlToolbox, hardwareType, "TargetPosition", "Number", identifier1, zero2, identifier22, zero2);
            ToolboxUtil.addDualPropertySetters(xmlToolbox, hardwareType, "ZeroPowerBehavior", "ZeroPowerBehavior", identifier1, zeroPowerBehavior, identifier22, zeroPowerBehavior);
            xmlToolbox.append(str);
        }
        List<HardwareItem> hardwareItemsForDcMotorEx = getHardwareItemsForDcMotorEx(hardwareItems);
        if (!hardwareItemsForDcMotorEx.isEmpty()) {
            String identifierForDcMotorEx = hardwareItemsForDcMotorEx.get(0).identifier;
            xmlToolbox.append("    <category name=\"Extended\">\n");
            properties.clear();
            SortedMap<String, String> properties3 = properties;
            properties3.put("TargetPositionTolerance", "Number");
            properties3.put("Velocity", "Number");
            setterValues.clear();
            setterValues.put("TargetPositionTolerance", new String[]{identifier});
            setterValues.put("Velocity", new String[]{identifier});
            ToolboxUtil.addProperties(xmlToolbox, hardwareType, identifierForDcMotorEx, properties3, setterValues, null);
            if (hardwareItemsForDcMotorEx.size() > 1) {
                String identifier1ForDcMotorEx = hardwareItemsForDcMotorEx.get(0).identifier;
                String identifier2ForDcMotorEx = hardwareItemsForDcMotorEx.get(1).identifier;
                xmlToolbox.append(str3);
                String identifier1ForDcMotorEx2 = identifier;
                zero = identifierForDcMotorEx;
                String str4 = identifier;
                ToolboxUtil.addDualPropertySetters(xmlToolbox, hardwareType, "TargetPositionTolerance", "Number", identifier1ForDcMotorEx, identifier1ForDcMotorEx2, identifier2ForDcMotorEx, str4);
                ToolboxUtil.addDualPropertySetters(xmlToolbox, hardwareType, "Velocity", "Number", identifier1ForDcMotorEx, identifier1ForDcMotorEx2, identifier2ForDcMotorEx, str4);
                xmlToolbox.append(str);
            } else {
                zero = identifierForDcMotorEx;
            }
            Map<String, Map<String, String>> functions3 = new LinkedHashMap<>();
            functions3.put("setMotorEnable", null);
            functions3.put("setMotorDisable", null);
            functions3.put("isMotorEnabled", null);
            Map<String, String> setVelocity_withAngleUnitArgs = new LinkedHashMap<>();
            setVelocity_withAngleUnitArgs.put("ANGULAR_RATE", identifier);
            setVelocity_withAngleUnitArgs.put("ANGLE_UNIT", angleUnit);
            functions3.put("setVelocity_withAngleUnit", setVelocity_withAngleUnitArgs);
            Map<String, String> getVelocity_withAngleUnitArgs = new LinkedHashMap<>();
            getVelocity_withAngleUnitArgs.put("ANGLE_UNIT", angleUnit);
            functions3.put("getVelocity_withAngleUnit", getVelocity_withAngleUnitArgs);
            Map<String, String> setVelocityPIDFCoefficientsArgs = new LinkedHashMap<>();
            setVelocityPIDFCoefficientsArgs.put("P", ToolboxUtil.makeNumberShadow(10));
            setVelocityPIDFCoefficientsArgs.put("I", ToolboxUtil.makeNumberShadow(10));
            setVelocityPIDFCoefficientsArgs.put("D", ToolboxUtil.makeNumberShadow(10));
            setVelocityPIDFCoefficientsArgs.put("F", ToolboxUtil.makeNumberShadow(10));
            functions3.put("setVelocityPIDFCoefficients", setVelocityPIDFCoefficientsArgs);
            Map<String, String> setPositionPIDFCoefficientsArgs = new LinkedHashMap<>();
            setPositionPIDFCoefficientsArgs.put("P", ToolboxUtil.makeNumberShadow(5));
            functions3.put("setPositionPIDFCoefficients", setPositionPIDFCoefficientsArgs);
            Map<String, String> setPIDFCoefficientsArgs = new LinkedHashMap<>();
            setPIDFCoefficientsArgs.put(str2, runModeRunUsingEncoder);
            setPIDFCoefficientsArgs.put("PIDF_COEFFICIENTS", ToolboxUtil.makeVariableGetBlock("pidfCoefficients"));
            functions3.put("setPIDFCoefficients", setPIDFCoefficientsArgs);
            Map<String, String> getPIDFCoefficientsArgs = new LinkedHashMap<>();
            getPIDFCoefficientsArgs.put(str2, runModeRunUsingEncoder);
            functions3.put("getPIDFCoefficients", getPIDFCoefficientsArgs);
            Map<String, String> getCurrentArgs = new LinkedHashMap<>();
            getCurrentArgs.put("CURRENT_UNIT", currentUnit);
            functions3.put("getCurrent", getCurrentArgs);
            Map<String, String> setCurrentAlertArgs = new LinkedHashMap<>();
            setCurrentAlertArgs.put("CURRENT", five);
            setCurrentAlertArgs.put("CURRENT_UNIT", currentUnit);
            functions3.put("setCurrentAlert", setCurrentAlertArgs);
            Map<String, String> getCurrentAlertArgs = new LinkedHashMap<>();
            getCurrentAlertArgs.put("CURRENT_UNIT", currentUnit);
            functions3.put("getCurrentAlert", getCurrentAlertArgs);
            functions3.put("isOverCurrent", null);
            ToolboxUtil.addFunctions(xmlToolbox, hardwareType, zero, functions3);
            xmlToolbox.append(str);
        }
    }

    private static void addDigitalChannelCategoryToToolbox(StringBuilder xmlToolbox, HardwareType hardwareType, List<HardwareItem> hardwareItems) {
        String identifier = hardwareItems.get(0).identifier;
        String mode = ToolboxUtil.makeTypedEnumShadow(hardwareType, "mode");
        SortedMap<String, String> properties = new TreeMap<>();
        properties.put("Mode", "DigitalChannelMode");
        properties.put("State", "Boolean");
        Map<String, String> enumBlocks = new HashMap<>();
        enumBlocks.put("Mode", ToolboxUtil.makeTypedEnumBlock(hardwareType, "mode"));
        Map<String, String[]> setterValues = new HashMap<>();
        setterValues.put("Mode", new String[]{mode});
        setterValues.put("State", new String[]{ToolboxUtil.makeBooleanShadow(true)});
        ToolboxUtil.addProperties(xmlToolbox, hardwareType, identifier, properties, setterValues, enumBlocks);
    }

    private static void addDistanceSensorCategoryToToolbox(StringBuilder xmlToolbox, HardwareType hardwareType, List<HardwareItem> hardwareItems) {
        HardwareItem hardwareItem = hardwareItems.get(0);
        String identifier = hardwareItem.identifier;
        Map<String, Map<String, String>> functions = new TreeMap<>();
        Map<String, String> getDistanceArgs = new LinkedHashMap<>();
        getDistanceArgs.put("DISTANCE_UNIT", ToolboxUtil.makeTypedEnumShadow(NotificationCompat.CATEGORY_NAVIGATION, "distanceUnit"));
        functions.put("getDistance", getDistanceArgs);
        ToolboxUtil.addFunctions(xmlToolbox, hardwareType, identifier, functions);
    }

    private static void addGoBildaPinpointCategoryToToolbox(StringBuilder xmlToolbox, HardwareType hardwareType, List<HardwareItem> hardwareItems) {
        String identifier = hardwareItems.get(0).identifier;
        String zero = ToolboxUtil.makeNumberShadow(0);
        String encoderDirection = ToolboxUtil.makeTypedEnumShadow(hardwareType, "encoderDirection");
        String readData = ToolboxUtil.makeTypedEnumShadow(hardwareType, "readData");
        String goBildaOdometryPods = ToolboxUtil.makeTypedEnumShadow(hardwareType, "goBildaOdometryPods");
        String distanceUnit = ToolboxUtil.makeTypedEnumShadow(NotificationCompat.CATEGORY_NAVIGATION, "distanceUnit");
        String mm = ToolboxUtil.makeTypedEnumShadow(NotificationCompat.CATEGORY_NAVIGATION, "distanceUnit", "DISTANCE_UNIT", "MM");
        String angleUnit = ToolboxUtil.makeTypedEnumShadow(NotificationCompat.CATEGORY_NAVIGATION, "angleUnit");
        String unnormalizedAngleUnit = ToolboxUtil.makeTypedEnumShadow(NotificationCompat.CATEGORY_NAVIGATION, "unnormalizedAngleUnit");
        SortedMap<String, String> properties = new TreeMap<>();
        properties.put("YawScalar", "Number");
        properties.put("DeviceID", "Number");
        properties.put("DeviceVersion", "Number");
        properties.put("LoopTime", "Number");
        properties.put("EncoderX", "Number");
        properties.put("EncoderY", "Number");
        properties.put("Frequency", "Number");
        properties.put("DeviceStatus", "DeviceStatus");
        properties.put("Position", "Pose2D");
        Map<String, String[]> setterValues = new HashMap<>();
        setterValues.put("YawScalar", new String[]{zero});
        Map<String, String> enumBlocks = new HashMap<>();
        enumBlocks.put("DeviceStatus", ToolboxUtil.makeTypedEnumBlock(hardwareType, "deviceStatus", "DEVICE_STATUS", "READY"));
        ToolboxUtil.addProperties(xmlToolbox, hardwareType, identifier, properties, setterValues, enumBlocks);
        Map<String, Map<String, String>> functions = new TreeMap<>();
        functions.put("update", null);
        Map<String, String> update_withReadDataArgs = new LinkedHashMap<>();
        update_withReadDataArgs.put("READ_DATA", readData);
        functions.put("update_withReadData", update_withReadDataArgs);
        Map<String, String> setOffsetsArgs = new LinkedHashMap<>();
        setOffsetsArgs.put("X_OFFSET", ToolboxUtil.makeNumberShadow(-104.0d));
        setOffsetsArgs.put("Y_OFFSET", ToolboxUtil.makeNumberShadow(-168.0d));
        setOffsetsArgs.put("DISTANCE_UNIT", distanceUnit);
        functions.put("setOffsets", setOffsetsArgs);
        functions.put("recalibrateIMU", null);
        functions.put("resetPosAndIMU", null);
        Map<String, String> setEncoderDirectionsArgs = new LinkedHashMap<>();
        setEncoderDirectionsArgs.put("X_ENCODER", encoderDirection);
        setEncoderDirectionsArgs.put("Y_ENCODER", encoderDirection);
        functions.put("setEncoderDirections", setEncoderDirectionsArgs);
        Map<String, String> setEncoderResolution_withPodsArgs = new LinkedHashMap<>();
        setEncoderResolution_withPodsArgs.put("PODS", goBildaOdometryPods);
        functions.put("setEncoderResolution_withPods", setEncoderResolution_withPodsArgs);
        Map<String, String> setEncoderResolution_withTicksArgs = new LinkedHashMap<>();
        setEncoderResolution_withTicksArgs.put("TICKS_PER_UNIT", ToolboxUtil.makeNumberShadow(13.26291192d));
        setEncoderResolution_withTicksArgs.put("DISTANCE_UNIT", mm);
        functions.put("setEncoderResolution_withTicks", setEncoderResolution_withTicksArgs);
        Map<String, String> setPositionArgs = new LinkedHashMap<>();
        setPositionArgs.put("POSITION", ToolboxUtil.makeVariableGetBlock("myPose2D"));
        functions.put("setPosition", setPositionArgs);
        Map<String, String> setPosXArgs = new LinkedHashMap<>();
        setPosXArgs.put("POS_X", zero);
        setPosXArgs.put("DISTANCE_UNIT", distanceUnit);
        functions.put("setPosX", setPosXArgs);
        Map<String, String> setPosYArgs = new LinkedHashMap<>();
        setPosYArgs.put("POS_Y", zero);
        setPosYArgs.put("DISTANCE_UNIT", distanceUnit);
        functions.put("setPosY", setPosYArgs);
        Map<String, String> setHeadingArgs = new LinkedHashMap<>();
        setHeadingArgs.put("HEADING", zero);
        setHeadingArgs.put("ANGLE_UNIT", angleUnit);
        functions.put("setHeading", setHeadingArgs);
        Map<String, String> getPosXArgs = new LinkedHashMap<>();
        getPosXArgs.put("DISTANCE_UNIT", distanceUnit);
        functions.put("getPosX", getPosXArgs);
        Map<String, String> getPosYArgs = new LinkedHashMap<>();
        getPosYArgs.put("DISTANCE_UNIT", distanceUnit);
        functions.put("getPosY", getPosYArgs);
        Map<String, String> getHeading_withAngleUnitArgs = new LinkedHashMap<>();
        getHeading_withAngleUnitArgs.put("ANGLE_UNIT", angleUnit);
        functions.put("getHeading_withAngleUnit", getHeading_withAngleUnitArgs);
        Map<String, String> getHeading_withUnnormalizedAngleUnitArgs = new LinkedHashMap<>();
        getHeading_withUnnormalizedAngleUnitArgs.put("UNNORMALIZED_ANGLE_UNIT", unnormalizedAngleUnit);
        functions.put("getHeading_withUnnormalizedAngleUnit", getHeading_withUnnormalizedAngleUnitArgs);
        Map<String, String> getVelXArgs = new LinkedHashMap<>();
        getVelXArgs.put("DISTANCE_UNIT", distanceUnit);
        functions.put("getVelX", getVelXArgs);
        Map<String, String> getVelYArgs = new LinkedHashMap<>();
        getVelYArgs.put("DISTANCE_UNIT", distanceUnit);
        functions.put("getVelY", getVelYArgs);
        Map<String, String> getHeadingVelocity_withUnnormalizedAngleUnitArgs = new LinkedHashMap<>();
        getHeadingVelocity_withUnnormalizedAngleUnitArgs.put("UNNORMALIZED_ANGLE_UNIT", unnormalizedAngleUnit);
        functions.put("getHeadingVelocity_withUnnormalizedAngleUnit", getHeadingVelocity_withUnnormalizedAngleUnitArgs);
        Map<String, String> getXOffsetArgs = new LinkedHashMap<>();
        getXOffsetArgs.put("DISTANCE_UNIT", distanceUnit);
        functions.put("getXOffset", getXOffsetArgs);
        Map<String, String> getYOffsetArgs = new LinkedHashMap<>();
        getYOffsetArgs.put("DISTANCE_UNIT", distanceUnit);
        functions.put("getYOffset", getYOffsetArgs);
        ToolboxUtil.addFunctions(xmlToolbox, hardwareType, identifier, functions);
    }

    private static void addGyroSensorCategoryToToolbox(StringBuilder xmlToolbox, HardwareType hardwareType, List<HardwareItem> hardwareItems) {
        String identifier = hardwareItems.get(0).identifier;
        String headingMode = ToolboxUtil.makeTypedEnumShadow(hardwareType, "headingMode");
        SortedMap<String, String> properties = new TreeMap<>();
        properties.put("Heading", "Number");
        properties.put("HeadingMode", "HeadingMode");
        properties.put("I2cAddress7Bit", "Number");
        properties.put("I2cAddress8Bit", "Number");
        properties.put("IntegratedZValue", "Number");
        properties.put("RawX", "Number");
        properties.put("RawY", "Number");
        properties.put("RawZ", "Number");
        properties.put("RotationFraction", "Number");
        properties.put("AngularVelocityAxes", "Array");
        properties.put("AngularOrientationAxes", "Array");
        Map<String, String> enumBlocks = new HashMap<>();
        enumBlocks.put("HeadingMode", ToolboxUtil.makeTypedEnumBlock(hardwareType, "headingMode"));
        Map<String, String[]> setterValues = new HashMap<>();
        setterValues.put("HeadingMode", new String[]{headingMode});
        setterValues.put("I2cAddress7Bit", new String[]{ToolboxUtil.makeNumberShadow(8)});
        setterValues.put("I2cAddress8Bit", new String[]{ToolboxUtil.makeNumberShadow(16)});
        ToolboxUtil.addProperties(xmlToolbox, hardwareType, identifier, properties, setterValues, enumBlocks);
        Map<String, Map<String, String>> functions = new TreeMap<>();
        functions.put("calibrate", null);
        functions.put("isCalibrating", null);
        functions.put("resetZAxisIntegrator", null);
        Map<String, String> getAngularVelocityArgs = new LinkedHashMap<>();
        getAngularVelocityArgs.put("ANGLE_UNIT", ToolboxUtil.makeTypedEnumShadow(NotificationCompat.CATEGORY_NAVIGATION, "angleUnit"));
        functions.put("getAngularVelocity", getAngularVelocityArgs);
        Map<String, String> getAngularOrientationArgs = new LinkedHashMap<>();
        getAngularOrientationArgs.put("AXES_REFERENCE", ToolboxUtil.makeTypedEnumShadow(NotificationCompat.CATEGORY_NAVIGATION, "axesReference"));
        getAngularOrientationArgs.put("AXES_ORDER", ToolboxUtil.makeTypedEnumShadow(NotificationCompat.CATEGORY_NAVIGATION, "axesOrder"));
        getAngularOrientationArgs.put("ANGLE_UNIT", ToolboxUtil.makeTypedEnumShadow(NotificationCompat.CATEGORY_NAVIGATION, "angleUnit"));
        functions.put("getAngularOrientation", getAngularOrientationArgs);
        ToolboxUtil.addFunctions(xmlToolbox, hardwareType, identifier, functions);
    }

    private static void addHuskyLensCategoryToToolbox(StringBuilder xmlToolbox, HardwareType hardwareType, List<HardwareItem> hardwareItems, AssetManager assetManager) throws IOException {
        String identifier = hardwareItems.get(0).identifier;
        Map<String, Map<String, String>> functions = new LinkedHashMap<>();
        Map<String, String> variableSetters = new HashMap<>();
        functions.put("knock", null);
        Map<String, String> selectAlgorithmArgs = new LinkedHashMap<>();
        selectAlgorithmArgs.put("ALGORITHM", ToolboxUtil.makeTypedEnumShadow(hardwareType, "algorithm", "ALGORITHM", "TAG_RECOGNITION"));
        functions.put("selectAlgorithm", selectAlgorithmArgs);
        functions.put("blocks", null);
        variableSetters.put("blocks", "myHuskyLensBlocks");
        Map<String, String> blocks_withIdArgs = new LinkedHashMap<>();
        blocks_withIdArgs.put("ID", ToolboxUtil.makeNumberShadow(1));
        functions.put("blocks_withId", blocks_withIdArgs);
        variableSetters.put("blocks_withId", "myHuskyLensBlocks");
        functions.put("arrows", null);
        variableSetters.put("arrows", "myHuskyLensArrows");
        Map<String, String> arrows_withIdArgs = new LinkedHashMap<>();
        arrows_withIdArgs.put("ID", ToolboxUtil.makeNumberShadow(1));
        functions.put("arrows_withId", arrows_withIdArgs);
        variableSetters.put("arrows_withId", "myHuskyLensArrows");
        ToolboxUtil.addFunctions(xmlToolbox, hardwareType, identifier, functions, null, variableSetters, null);
        if (assetManager != null) {
            addAsset(xmlToolbox, assetManager, "toolbox/husky_lens_block.xml");
            addAsset(xmlToolbox, assetManager, "toolbox/husky_lens_arrow.xml");
        }
    }

    private static void addIrSeekerSensorCategoryToToolbox(StringBuilder xmlToolbox, HardwareType hardwareType, List<HardwareItem> hardwareItems) {
        String identifier = hardwareItems.get(0).identifier;
        String mode = ToolboxUtil.makeTypedEnumShadow(hardwareType, "mode");
        String threshold = ToolboxUtil.makeNumberShadow(0.003d);
        SortedMap<String, String> properties = new TreeMap<>();
        properties.put("SignalDetectedThreshold", "Number");
        properties.put("Mode", "IrSeekerSensorMode");
        properties.put("IsSignalDetected", "Boolean");
        properties.put("Angle", "Number");
        properties.put("Strength", "Number");
        properties.put("I2cAddress7Bit", "Number");
        properties.put("I2cAddress8Bit", "Number");
        Map<String, String> enumBlocks = new HashMap<>();
        enumBlocks.put("Mode", ToolboxUtil.makeTypedEnumBlock(hardwareType, "mode"));
        Map<String, String[]> setterValues = new HashMap<>();
        setterValues.put("SignalDetectedThreshold", new String[]{threshold});
        setterValues.put("Mode", new String[]{mode});
        setterValues.put("I2cAddress7Bit", new String[]{ToolboxUtil.makeNumberShadow(8)});
        setterValues.put("I2cAddress8Bit", new String[]{ToolboxUtil.makeNumberShadow(16)});
        ToolboxUtil.addProperties(xmlToolbox, hardwareType, identifier, properties, setterValues, enumBlocks);
    }

    private static void addLedCategoryToToolbox(StringBuilder xmlToolbox, HardwareType hardwareType, List<HardwareItem> hardwareItems) {
        String identifier = hardwareItems.get(0).identifier;
        Map<String, Map<String, String>> functions = new TreeMap<>();
        Map<String, String> enableLedArgs = new LinkedHashMap<>();
        enableLedArgs.put("ENABLE", ToolboxUtil.makeBooleanShadow(true));
        functions.put("enableLed_Boolean", enableLedArgs);
        functions.put("isLightOn", null);
        functions.put("on", null);
        functions.put("off", null);
        ToolboxUtil.addFunctions(xmlToolbox, hardwareType, identifier, functions);
    }

    private static void addLimelight3ACategoryToToolbox(StringBuilder xmlToolbox, HardwareType hardwareType, List<HardwareItem> hardwareItems, AssetManager assetManager) throws IOException {
        String identifier = hardwareItems.get(0).identifier;
        Map<String, Map<String, String>> functions = new TreeMap<>();
        functions.put("start", null);
        functions.put("pause", null);
        functions.put("stop", null);
        functions.put("isRunning", null);
        Map<String, String> setPollRateHzArgs = new LinkedHashMap<>();
        setPollRateHzArgs.put("RATE_HZ", ToolboxUtil.makeNumberShadow(100));
        functions.put("setPollRateHz", setPollRateHzArgs);
        functions.put("getTimeSinceLastUpdate", null);
        functions.put("isConnected", null);
        functions.put("getLatestResult", null);
        functions.put("getStatus", null);
        functions.put("reloadPipeline", null);
        Map<String, String> pipelineSwitchArgs = new LinkedHashMap<>();
        pipelineSwitchArgs.put("INDEX", ToolboxUtil.makeNumberShadow(0));
        functions.put("pipelineSwitch", pipelineSwitchArgs);
        Map<String, String> updatePythonInputs_with8DoublesArgs = new LinkedHashMap<>();
        updatePythonInputs_with8DoublesArgs.put("INPUT1", ToolboxUtil.makeNumberShadow(1));
        updatePythonInputs_with8DoublesArgs.put("INPUT2", ToolboxUtil.makeNumberShadow(2));
        updatePythonInputs_with8DoublesArgs.put("INPUT3", ToolboxUtil.makeNumberShadow(3));
        updatePythonInputs_with8DoublesArgs.put("INPUT4", ToolboxUtil.makeNumberShadow(4));
        updatePythonInputs_with8DoublesArgs.put("INPUT5", ToolboxUtil.makeNumberShadow(5));
        updatePythonInputs_with8DoublesArgs.put("INPUT6", ToolboxUtil.makeNumberShadow(6));
        updatePythonInputs_with8DoublesArgs.put("INPUT7", ToolboxUtil.makeNumberShadow(7));
        updatePythonInputs_with8DoublesArgs.put("INPUT8", ToolboxUtil.makeNumberShadow(8));
        functions.put("updatePythonInputs_with8Doubles", updatePythonInputs_with8DoublesArgs);
        Map<String, String> updatePythonInputs_withArray = new LinkedHashMap<>();
        updatePythonInputs_withArray.put("INPUTS", ToolboxUtil.makeVariableGetBlock("pythonInputs"));
        functions.put("updatePythonInputs_withArray", updatePythonInputs_withArray);
        Map<String, String> updateRobotOrientationArgs = new LinkedHashMap<>();
        updateRobotOrientationArgs.put("YAW", ToolboxUtil.makeNumberShadow(45));
        functions.put("updateRobotOrientation", updateRobotOrientationArgs);
        functions.put("shutdown", null);
        ToolboxUtil.addFunctions(xmlToolbox, hardwareType, identifier, functions);
        if (assetManager != null) {
            addAsset(xmlToolbox, assetManager, "toolbox/limelight.xml");
        }
    }

    private static void addLightSensorCategoryToToolbox(StringBuilder xmlToolbox, HardwareType hardwareType, List<HardwareItem> hardwareItems) {
        String identifier = hardwareItems.get(0).identifier;
        SortedMap<String, String> properties = new TreeMap<>();
        properties.put("LightDetected", "Number");
        properties.put("RawLightDetected", "Number");
        properties.put("RawLightDetectedMax", "Number");
        ToolboxUtil.addProperties(xmlToolbox, hardwareType, identifier, properties, null, null);
        Map<String, Map<String, String>> functions = new TreeMap<>();
        Map<String, String> enableLedArgs = new LinkedHashMap<>();
        enableLedArgs.put("ENABLE", ToolboxUtil.makeBooleanShadow(true));
        functions.put("enableLed_Boolean", enableLedArgs);
        ToolboxUtil.addFunctions(xmlToolbox, hardwareType, identifier, functions);
    }

    private static void addColorRangeSensorCategoryToToolbox(StringBuilder xmlToolbox, HardwareType hardwareType, List<HardwareItem> hardwareItems) {
        String identifier = hardwareItems.get(0).identifier;
        SortedMap<String, String> properties = new TreeMap<>();
        properties.put("Red", "Number");
        properties.put("Green", "Number");
        properties.put("Blue", "Number");
        properties.put("Alpha", "Number");
        properties.put("Argb", "Number");
        properties.put("Gain", "Number");
        properties.put("I2cAddress7Bit", "Number");
        properties.put("I2cAddress8Bit", "Number");
        properties.put("LightDetected", "Number");
        properties.put("RawLightDetected", "Number");
        properties.put("RawLightDetectedMax", "Number");
        Map<String, String[]> setterValues = new HashMap<>();
        setterValues.put("Gain", new String[]{ToolboxUtil.makeNumberShadow(2)});
        setterValues.put("I2cAddress7Bit", new String[]{ToolboxUtil.makeNumberShadow(8)});
        setterValues.put("I2cAddress8Bit", new String[]{ToolboxUtil.makeNumberShadow(16)});
        ToolboxUtil.addProperties(xmlToolbox, hardwareType, identifier, properties, setterValues, null);
        Map<String, Map<String, String>> functions = new TreeMap<>();
        Map<String, String> getDistanceArgs = new LinkedHashMap<>();
        getDistanceArgs.put("UNIT", ToolboxUtil.makeTypedEnumShadow(NotificationCompat.CATEGORY_NAVIGATION, "distanceUnit"));
        functions.put("getDistance_Number", getDistanceArgs);
        functions.put("getNormalizedColors", null);
        ToolboxUtil.addFunctions(xmlToolbox, hardwareType, identifier, functions);
    }

    private static void addMaxSonarI2CXLCategoryToToolbox(StringBuilder xmlToolbox, HardwareType hardwareType, List<HardwareItem> hardwareItems) {
        String identifier = hardwareItems.get(0).identifier;
        SortedMap<String, String> properties = new TreeMap<>();
        properties.put("I2cAddress7Bit", "Number");
        properties.put("I2cAddress8Bit", "Number");
        Map<String, String[]> setterValues = new HashMap<>();
        setterValues.put("I2cAddress7Bit", new String[]{ToolboxUtil.makeNumberShadow(8)});
        setterValues.put("I2cAddress8Bit", new String[]{ToolboxUtil.makeNumberShadow(16)});
        ToolboxUtil.addProperties(xmlToolbox, hardwareType, identifier, properties, setterValues, null);
        Map<String, Map<String, String>> functions = new TreeMap<>();
        Map<String, String> getDistanceSyncArgs = new LinkedHashMap<>();
        getDistanceSyncArgs.put("DISTANCE_UNIT", ToolboxUtil.makeTypedEnumShadow(NotificationCompat.CATEGORY_NAVIGATION, "distanceUnit"));
        functions.put("getDistanceSync", getDistanceSyncArgs);
        Map<String, String> getDistanceSync_withDelayArgs = new LinkedHashMap<>();
        getDistanceSync_withDelayArgs.put("DELAY", ToolboxUtil.makeNumberShadow(50));
        getDistanceSync_withDelayArgs.put("DISTANCE_UNIT", ToolboxUtil.makeTypedEnumShadow(NotificationCompat.CATEGORY_NAVIGATION, "distanceUnit"));
        functions.put("getDistanceSync_withDelay", getDistanceSync_withDelayArgs);
        Map<String, String> getDistanceAsyncArgs = new LinkedHashMap<>();
        getDistanceAsyncArgs.put("DISTANCE_UNIT", ToolboxUtil.makeTypedEnumShadow(NotificationCompat.CATEGORY_NAVIGATION, "distanceUnit"));
        functions.put("getDistanceAsync", getDistanceAsyncArgs);
        Map<String, String> getDistanceAsync_withDelayArgs = new LinkedHashMap<>();
        getDistanceAsync_withDelayArgs.put("DELAY", ToolboxUtil.makeNumberShadow(50));
        getDistanceAsync_withDelayArgs.put("DISTANCE_UNIT", ToolboxUtil.makeTypedEnumShadow(NotificationCompat.CATEGORY_NAVIGATION, "distanceUnit"));
        functions.put("getDistanceAsync_withDelay", getDistanceAsync_withDelayArgs);
        Map<String, String> writeI2cAddrToSensorEEPROMArgs = new LinkedHashMap<>();
        writeI2cAddrToSensorEEPROMArgs.put("ADDRESS", ToolboxUtil.makeNumberShadow(20));
        functions.put("writeI2cAddrToSensorEEPROM", writeI2cAddrToSensorEEPROMArgs);
        ToolboxUtil.addFunctions(xmlToolbox, hardwareType, identifier, functions);
    }

    private static void addMrI2cCompassSensorCategoryToToolbox(StringBuilder xmlToolbox, HardwareType hardwareType, List<HardwareItem> hardwareItems) {
        String identifier = hardwareItems.get(0).identifier;
        SortedMap<String, String> properties = new TreeMap<>();
        properties.put("Direction", "Number");
        properties.put("XAccel", "Number");
        properties.put("YAccel", "Number");
        properties.put("ZAccel", "Number");
        properties.put("XMagneticFlux", "Number");
        properties.put("YMagneticFlux", "Number");
        properties.put("ZMagneticFlux", "Number");
        properties.put("I2cAddress7Bit", "Number");
        properties.put("I2cAddress8Bit", "Number");
        Map<String, String[]> setterValues = new HashMap<>();
        setterValues.put("I2cAddress7Bit", new String[]{ToolboxUtil.makeNumberShadow(8)});
        setterValues.put("I2cAddress8Bit", new String[]{ToolboxUtil.makeNumberShadow(16)});
        ToolboxUtil.addProperties(xmlToolbox, hardwareType, identifier, properties, setterValues, null);
        Map<String, Map<String, String>> functions = new TreeMap<>();
        Map<String, String> setModeArgs = new LinkedHashMap<>();
        setModeArgs.put("COMPASS_MODE", ToolboxUtil.makeTypedEnumShadow(hardwareType, "compassMode"));
        functions.put("setMode_CompassMode", setModeArgs);
        functions.put("isCalibrating", null);
        functions.put("calibrationFailed", null);
        ToolboxUtil.addFunctions(xmlToolbox, hardwareType, identifier, functions);
    }

    private static void addMrI2cRangeSensorCategoryToToolbox(StringBuilder xmlToolbox, HardwareType hardwareType, List<HardwareItem> hardwareItems) {
        String identifier = hardwareItems.get(0).identifier;
        SortedMap<String, String> properties = new TreeMap<>();
        properties.put("LightDetected", "Number");
        properties.put("RawLightDetected", "Number");
        properties.put("RawLightDetectedMax", "Number");
        properties.put("RawUltrasonic", "Number");
        properties.put("RawOptical", "Number");
        properties.put("CmUltrasonic", "Number");
        properties.put("CmOptical", "Number");
        properties.put("I2cAddress7Bit", "Number");
        properties.put("I2cAddress8Bit", "Number");
        Map<String, String[]> setterValues = new HashMap<>();
        setterValues.put("I2cAddress7Bit", new String[]{ToolboxUtil.makeNumberShadow(8)});
        setterValues.put("I2cAddress8Bit", new String[]{ToolboxUtil.makeNumberShadow(16)});
        ToolboxUtil.addProperties(xmlToolbox, hardwareType, identifier, properties, setterValues, null);
        Map<String, Map<String, String>> functions = new TreeMap<>();
        Map<String, String> getDistanceArgs = new LinkedHashMap<>();
        getDistanceArgs.put("UNIT", ToolboxUtil.makeTypedEnumShadow(hardwareType, "distanceUnit"));
        functions.put("getDistance_Number", getDistanceArgs);
        ToolboxUtil.addFunctions(xmlToolbox, hardwareType, identifier, functions);
    }

    private static void addOctoQuadCategoryToToolbox(StringBuilder xmlToolbox, HardwareType hardwareType, List<HardwareItem> hardwareItems, AssetManager assetManager) throws IOException {
        String identifier = hardwareItems.get(0).identifier;
        String channelBankConfig = ToolboxUtil.makeTypedEnumShadow(hardwareType, "channelBankConfig");
        String i2cRecoveryMode = ToolboxUtil.makeTypedEnumShadow(hardwareType, "i2cRecoveryMode");
        String zero = ToolboxUtil.makeNumberShadow(0);
        String one = ToolboxUtil.makeNumberShadow(1);
        String ticks = ToolboxUtil.makeNumberShadow(2.9917d);
        String offset = ToolboxUtil.makeNumberShadow(1.2345d);
        if (assetManager != null) {
            addAsset(xmlToolbox, assetManager, "toolbox/octoquad.xml");
        }
        SortedMap<String, String> properties = new TreeMap<>();
        Map<String, String> enumBlocks = new HashMap<>();
        Map<String, String[]> setterValues = new HashMap<>();
        properties.put("ChipId", "Number");
        properties.put("FirmwareVersionString", "String");
        properties.put("ChannelBankConfig", "ChannelBankConfig");
        enumBlocks.put("ChannelBankConfig", ToolboxUtil.makeTypedEnumBlock(hardwareType, "channelBankConfig"));
        properties.put("I2cRecoveryMode", "I2cRecoveryMode");
        enumBlocks.put("I2cRecoveryMode", ToolboxUtil.makeTypedEnumBlock(hardwareType, "i2cRecoveryMode"));
        properties.put("LocalizerHeadingAxisChoice", "LocalizerYawAxis");
        enumBlocks.put("LocalizerHeadingAxisChoice", ToolboxUtil.makeTypedEnumBlock(hardwareType, "localizerYawAxis", "LOCALIZER_YAW_AXIS", "X"));
        properties.put("LocalizerStatus", "LocalizerStatus");
        enumBlocks.put("LocalizerStatus", ToolboxUtil.makeTypedEnumBlock(hardwareType, "localizerStatus", "LOCALIZER_STATUS", "RUNNING"));
        setterValues.put("ChannelBankConfig", new String[]{channelBankConfig});
        setterValues.put("I2cRecoveryMode", new String[]{i2cRecoveryMode});
        ToolboxUtil.addProperties(xmlToolbox, hardwareType, identifier, properties, setterValues, enumBlocks);
        enumBlocks.clear();
        Map<String, Map<String, String>> functions = new TreeMap<>();
        Map<String, String> variableSetters = new HashMap<>();
        functions.put("resetAllPositions", null);
        Map<String, String> resetSinglePositionArgs = new LinkedHashMap<>();
        resetSinglePositionArgs.put("INDEX", zero);
        functions.put("resetSinglePosition", resetSinglePositionArgs);
        Map<String, String> setSingleEncoderDirectionArgs = new LinkedHashMap<>();
        setSingleEncoderDirectionArgs.put("INDEX", zero);
        String i2cRecoveryMode2 = ToolboxUtil.makeTypedEnumShadow(hardwareType, "encoderDirection");
        setSingleEncoderDirectionArgs.put("DIRECTION", i2cRecoveryMode2);
        functions.put("setSingleEncoderDirection", setSingleEncoderDirectionArgs);
        Map<String, String> getSingleEncoderDirectionArgs = new LinkedHashMap<>();
        getSingleEncoderDirectionArgs.put("INDEX", zero);
        enumBlocks.put("getSingleEncoderDirection", ToolboxUtil.makeTypedEnumBlock(hardwareType, "encoderDirection"));
        functions.put("getSingleEncoderDirection", getSingleEncoderDirectionArgs);
        Map<String, String> setAllVelocitySampleIntervalsArgs = new LinkedHashMap<>();
        setAllVelocitySampleIntervalsArgs.put("INTERVAL_MS", ToolboxUtil.makeNumberShadow(24));
        functions.put("setAllVelocitySampleIntervals", setAllVelocitySampleIntervalsArgs);
        Map<String, String> setSingleVelocitySampleIntervalArgs = new LinkedHashMap<>();
        setSingleVelocitySampleIntervalArgs.put("INDEX", zero);
        setSingleVelocitySampleIntervalArgs.put("INTERVAL_MS", ToolboxUtil.makeNumberShadow(25));
        functions.put("setSingleVelocitySampleInterval", setSingleVelocitySampleIntervalArgs);
        Map<String, String> getSingleVelocitySampleIntervalArgs = new LinkedHashMap<>();
        getSingleVelocitySampleIntervalArgs.put("INDEX", zero);
        functions.put("getSingleVelocitySampleInterval", getSingleVelocitySampleIntervalArgs);
        Map<String, String> setSingleChannelPulseWidthParamsArgs = new LinkedHashMap<>();
        setSingleChannelPulseWidthParamsArgs.put("INDEX", zero);
        setSingleChannelPulseWidthParamsArgs.put("MIN_WIDTH_US", ToolboxUtil.makeNumberShadow(1));
        setSingleChannelPulseWidthParamsArgs.put("MAX_WIDTH_US", ToolboxUtil.makeNumberShadow(1024));
        functions.put("setSingleChannelPulseWidthParams", setSingleChannelPulseWidthParamsArgs);
        functions.put("resetEverything", null);
        functions.put("saveParametersToFlash", null);
        Map<String, String> setCachingModeArgs = new LinkedHashMap<>();
        setCachingModeArgs.put("MODE", ToolboxUtil.makeTypedEnumShadow(hardwareType, "cachingMode", "CACHING_MODE", "AUTO"));
        functions.put("setCachingMode", setCachingModeArgs);
        functions.put("refreshCache", null);
        Map<String, String> readSinglePosition_CachingArgs = new LinkedHashMap<>();
        readSinglePosition_CachingArgs.put("INDEX", zero);
        functions.put("readSinglePosition_Caching", readSinglePosition_CachingArgs);
        Map<String, String> readSingleVelocity_CachingArgs = new LinkedHashMap<>();
        readSingleVelocity_CachingArgs.put("INDEX", zero);
        functions.put("readSingleVelocity_Caching", readSingleVelocity_CachingArgs);
        Map<String, String> setSingleChannelPulseWidthTracksWrapArgs = new LinkedHashMap<>();
        setSingleChannelPulseWidthTracksWrapArgs.put("INDEX", zero);
        setSingleChannelPulseWidthTracksWrapArgs.put("TRACK_WRAP", ToolboxUtil.makeBooleanShadow(true));
        functions.put("setSingleChannelPulseWidthTracksWrap", setSingleChannelPulseWidthTracksWrapArgs);
        Map<String, String> getSingleChannelPulseWidthTracksWrapArgs = new LinkedHashMap<>();
        getSingleChannelPulseWidthTracksWrapArgs.put("INDEX", zero);
        functions.put("getSingleChannelPulseWidthTracksWrap", getSingleChannelPulseWidthTracksWrapArgs);
        Map<String, String> setAllLocalizerParametersArgs = new LinkedHashMap<>();
        setAllLocalizerParametersArgs.put("PORT_X", zero);
        setAllLocalizerParametersArgs.put("PORT_Y", zero);
        setAllLocalizerParametersArgs.put("TICKS_PER_MM_X", ticks);
        setAllLocalizerParametersArgs.put("TICKS_PER_MM_Y", ticks);
        setAllLocalizerParametersArgs.put("TCP_OFFSET_MM_X", offset);
        setAllLocalizerParametersArgs.put("TCP_OFFSET_MM_Y", offset);
        setAllLocalizerParametersArgs.put("HEADING_SCALAR", one);
        setAllLocalizerParametersArgs.put("VELOCITY_INTERVAL_MS", ToolboxUtil.makeNumberShadow(128));
        functions.put("setAllLocalizerParameters", setAllLocalizerParametersArgs);
        functions.put("readLocalizerData", null);
        variableSetters.put("readLocalizerData", "myLocalizerDataBlock");
        Map<String, String> setLocalizerPoseArgs = new LinkedHashMap<>();
        setLocalizerPoseArgs.put("POS_X_MM", zero);
        setLocalizerPoseArgs.put("POS_Y_MM", zero);
        setLocalizerPoseArgs.put("HEADING_RAD", one);
        functions.put("setLocalizerPose", setLocalizerPoseArgs);
        Map<String, String> setLocalizerHeadingArgs = new LinkedHashMap<>();
        setLocalizerHeadingArgs.put("HEADING_RAD", one);
        functions.put("setLocalizerHeading", setLocalizerHeadingArgs);
        functions.put("resetLocalizerAndCalibrateIMU", null);
        ToolboxUtil.addFunctions(xmlToolbox, hardwareType, identifier, functions, null, variableSetters, enumBlocks);
        if (assetManager != null) {
            addAsset(xmlToolbox, assetManager, "toolbox/octoquad_localizer_data_block.xml");
        }
    }

    private static void addOpticalDistanceSensorCategoryToToolbox(StringBuilder xmlToolbox, HardwareType hardwareType, List<HardwareItem> hardwareItems) {
        String identifier = hardwareItems.get(0).identifier;
        SortedMap<String, String> properties = new TreeMap<>();
        properties.put("LightDetected", "Number");
        properties.put("RawLightDetected", "Number");
        properties.put("RawLightDetectedMax", "Number");
        ToolboxUtil.addProperties(xmlToolbox, hardwareType, identifier, properties, null, null);
        Map<String, Map<String, String>> functions = new TreeMap<>();
        Map<String, String> enableLedArgs = new LinkedHashMap<>();
        enableLedArgs.put("ENABLE", ToolboxUtil.makeBooleanShadow(true));
        functions.put("enableLed_Boolean", enableLedArgs);
        ToolboxUtil.addFunctions(xmlToolbox, hardwareType, identifier, functions);
    }

    private static void addRevBlinkinLedDriverCategoryToToolbox(StringBuilder xmlToolbox, AssetManager assetManager) throws IOException {
        addAsset(xmlToolbox, assetManager, "toolbox/rev_blinkin_led_driver.xml");
    }

    private static void addServoCategoryToToolbox(StringBuilder xmlToolbox, HardwareType hardwareType, List<HardwareItem> hardwareItems) {
        String identifier = hardwareItems.get(0).identifier;
        SortedMap<String, String> properties = new TreeMap<>();
        properties.put("Direction", "Direction");
        properties.put("Position", "Number");
        Map<String, String> enumBlocks = new HashMap<>();
        enumBlocks.put("Direction", ToolboxUtil.makeTypedEnumBlock(hardwareType, "direction"));
        Map<String, String[]> setterValues = new HashMap<>();
        setterValues.put("Direction", new String[]{ToolboxUtil.makeTypedEnumShadow(hardwareType, "direction")});
        setterValues.put("Position", new String[]{ToolboxUtil.makeNumberShadow(0)});
        ToolboxUtil.addProperties(xmlToolbox, hardwareType, identifier, properties, setterValues, enumBlocks);
        Map<String, Map<String, String>> functions = new TreeMap<>();
        Map<String, String> scaleRangeArgs = new LinkedHashMap<>();
        scaleRangeArgs.put("MIN", ToolboxUtil.makeNumberShadow(0.2d));
        scaleRangeArgs.put("MAX", ToolboxUtil.makeNumberShadow(0.8d));
        functions.put("scaleRange_Number", scaleRangeArgs);
        functions.put("setPwmEnable", null);
        functions.put("setPwmDisable", null);
        functions.put("isPwmEnabled", null);
        ToolboxUtil.addFunctions(xmlToolbox, hardwareType, identifier, functions);
    }

    private static void addServoControllerCategoryToToolbox(StringBuilder xmlToolbox, HardwareType hardwareType, List<HardwareItem> hardwareItems) {
        String identifier = hardwareItems.get(0).identifier;
        SortedMap<String, String> properties = new TreeMap<>();
        properties.put("PwmStatus", "PwmStatus");
        ToolboxUtil.addProperties(xmlToolbox, hardwareType, identifier, properties, null, null);
        Map<String, Map<String, String>> functions = new TreeMap<>();
        functions.put("pwmEnable", null);
        functions.put("pwmDisable", null);
        ToolboxUtil.addFunctions(xmlToolbox, hardwareType, identifier, functions);
    }

    private static void addSparkFunLEDStickCategoryToToolbox(StringBuilder xmlToolbox, HardwareType hardwareType, List<HardwareItem> hardwareItems, AssetManager assetManager) throws IOException {
        String identifier = hardwareItems.get(0).identifier;
        ToolboxUtil.makeNumberShadow(0);
        String one = ToolboxUtil.makeNumberShadow(1);
        String twenty = ToolboxUtil.makeNumberShadow(20);
        Map<String, Map<String, String>> functions = new TreeMap<>();
        Map<String, String> setColor_withPositionArgs = new LinkedHashMap<>();
        setColor_withPositionArgs.put("POSITION", one);
        setColor_withPositionArgs.put("COLOR", "<shadow type=\"color_constant_Number\"><field name=\"CONSTANT\">BLUE</field></shadow>\n");
        functions.put("setColor_withPosition", setColor_withPositionArgs);
        Map<String, String> setColorArgs = new LinkedHashMap<>();
        setColorArgs.put("COLOR", "<shadow type=\"color_constant_Number\"><field name=\"CONSTANT\">BLUE</field></shadow>\n");
        functions.put("setColor", setColorArgs);
        Map<String, String> setColorsArgs = new LinkedHashMap<>();
        setColorsArgs.put("COLORS", ToolboxUtil.makeVariableGetBlock("ledColors"));
        functions.put("setColors", setColorsArgs);
        Map<String, String> setBrightness_withPositionArgs = new LinkedHashMap<>();
        setBrightness_withPositionArgs.put("POSITION", one);
        setBrightness_withPositionArgs.put("BRIGHTNESS", twenty);
        functions.put("setBrightness_withPosition", setBrightness_withPositionArgs);
        Map<String, String> setBrightnessArgs = new LinkedHashMap<>();
        setBrightnessArgs.put("BRIGHTNESS", twenty);
        functions.put("setBrightness", setBrightnessArgs);
        functions.put("turnAllOff", null);
        ToolboxUtil.addFunctions(xmlToolbox, hardwareType, identifier, functions);
    }

    private static void addSparkFunOTOSCategoryToToolbox(StringBuilder xmlToolbox, HardwareType hardwareType, List<HardwareItem> hardwareItems, AssetManager assetManager) throws IOException {
        String identifier = hardwareItems.get(0).identifier;
        String one = ToolboxUtil.makeNumberShadow(1);
        if (assetManager != null) {
            addAsset(xmlToolbox, assetManager, "toolbox/sparkfun_otos.xml");
        }
        SortedMap<String, String> properties = new TreeMap<>();
        properties.put("Status", "Status");
        properties.put("Offset", "Pose2D");
        properties.put("Position", "Pose2D");
        properties.put("Acceleration", "Pose2D");
        properties.put("Velocity", "Pose2D");
        properties.put("PositionStdDev", "Pose2D");
        properties.put("AccelerationStdDev", "Pose2D");
        properties.put("VelocityStdDev", "Pose2D");
        properties.put("SignalProcessConfig", "SignalProcessConfig");
        properties.put("ImuCalibrationProgress", "Number");
        properties.put("LinearUnit", "DistanceUnit");
        properties.put("LinearScalar", "Number_ReturnBoolean");
        properties.put("AngularScalar", "Number_ReturnBoolean");
        properties.put("AngularUnit", "AngleUnit");
        Map<String, String[]> setterValues = new HashMap<>();
        setterValues.put("Offset", new String[]{ToolboxUtil.makeVariableGetBlock("myPose")});
        setterValues.put("Position", new String[]{ToolboxUtil.makeVariableGetBlock("myPose")});
        setterValues.put("SignalProcessConfig", new String[]{ToolboxUtil.makeVariableGetBlock("mySignalProcessConfig")});
        setterValues.put("LinearScalar", new String[]{one});
        setterValues.put("LinearUnit", new String[]{ToolboxUtil.makeTypedEnumShadow(NotificationCompat.CATEGORY_NAVIGATION, "distanceUnit")});
        setterValues.put("AngularScalar", new String[]{one});
        setterValues.put("AngularUnit", new String[]{ToolboxUtil.makeTypedEnumShadow(NotificationCompat.CATEGORY_NAVIGATION, "angleUnit")});
        ToolboxUtil.addProperties(xmlToolbox, hardwareType, identifier, properties, setterValues, null);
        Map<String, Map<String, String>> functions = new TreeMap<>();
        Map<String, String> getPosVelAccArgs = new LinkedHashMap<>();
        getPosVelAccArgs.put("POS", ToolboxUtil.makeVariableGetBlock("pos"));
        getPosVelAccArgs.put("VEL", ToolboxUtil.makeVariableGetBlock("vel"));
        getPosVelAccArgs.put("ACC", ToolboxUtil.makeVariableGetBlock("acc"));
        functions.put("getPosVelAcc", getPosVelAccArgs);
        Map<String, String> getPosVelAccStdDevArgs = new LinkedHashMap<>();
        getPosVelAccStdDevArgs.put("POS_STD_DEV", ToolboxUtil.makeVariableGetBlock("posStdDev"));
        getPosVelAccStdDevArgs.put("VEL_STD_DEV", ToolboxUtil.makeVariableGetBlock("velStdDev"));
        getPosVelAccStdDevArgs.put("ACC_STD_DEV", ToolboxUtil.makeVariableGetBlock("accStdDev"));
        functions.put("getPosVelAccStdDev", getPosVelAccStdDevArgs);
        Map<String, String> getPosVelAccAndStdDevArgs = new LinkedHashMap<>();
        getPosVelAccAndStdDevArgs.put("POS", ToolboxUtil.makeVariableGetBlock("pos"));
        getPosVelAccAndStdDevArgs.put("VEL", ToolboxUtil.makeVariableGetBlock("vel"));
        getPosVelAccAndStdDevArgs.put("ACC", ToolboxUtil.makeVariableGetBlock("acc"));
        getPosVelAccAndStdDevArgs.put("POS_STD_DEV", ToolboxUtil.makeVariableGetBlock("posStdDev"));
        getPosVelAccAndStdDevArgs.put("VEL_STD_DEV", ToolboxUtil.makeVariableGetBlock("velStdDev"));
        getPosVelAccAndStdDevArgs.put("ACC_STD_DEV", ToolboxUtil.makeVariableGetBlock("accStdDev"));
        functions.put("getPosVelAccAndStdDev", getPosVelAccAndStdDevArgs);
        Map<String, String> getVersionInfoArgs = new LinkedHashMap<>();
        getVersionInfoArgs.put("HW_VERSION", ToolboxUtil.makeVariableGetBlock("hwVersion"));
        getVersionInfoArgs.put("FW_VERSION", ToolboxUtil.makeVariableGetBlock("fwVersion"));
        functions.put("getVersionInfo", getVersionInfoArgs);
        functions.put("begin", null);
        functions.put("calibrateImu", null);
        Map<String, String> calibrateImuArgs = new LinkedHashMap<>();
        calibrateImuArgs.put("NUM_SAMPLES", ToolboxUtil.makeNumberShadow(255));
        calibrateImuArgs.put("WAIT_UNTIL_DONE", ToolboxUtil.makeBooleanShadow(true));
        functions.put("calibrateImu_withArgs", calibrateImuArgs);
        functions.put("isConnected", null);
        functions.put("selfTest", null);
        functions.put("resetTracking", null);
        ToolboxUtil.addFunctions(xmlToolbox, hardwareType, identifier, functions);
    }

    private static void addTouchSensorCategoryToToolbox(StringBuilder xmlToolbox, HardwareType hardwareType, List<HardwareItem> hardwareItems) {
        String identifier = hardwareItems.get(0).identifier;
        SortedMap<String, String> properties = new TreeMap<>();
        properties.put("IsPressed", "Boolean");
        ToolboxUtil.addProperties(xmlToolbox, hardwareType, identifier, properties, null, null);
    }

    private static void addUltrasonicSensorCategoryToToolbox(StringBuilder xmlToolbox, HardwareType hardwareType, List<HardwareItem> hardwareItems) {
        String identifier = hardwareItems.get(0).identifier;
        SortedMap<String, String> properties = new TreeMap<>();
        properties.put("UltrasonicLevel", "Number");
        ToolboxUtil.addProperties(xmlToolbox, hardwareType, identifier, properties, null, null);
    }

    private static void addVoltageSensorCategoryToToolbox(StringBuilder xmlToolbox, HardwareType hardwareType, List<HardwareItem> hardwareItems) {
        String identifier = hardwareItems.get(0).identifier;
        SortedMap<String, String> properties = new TreeMap<>();
        properties.put("Voltage", "Number");
        ToolboxUtil.addProperties(xmlToolbox, hardwareType, identifier, properties, null, null);
    }

    public static String upgradeJs(String jsContent, HardwareItemMap hardwareItemMap) {
        return replaceIdentifierInJs(replaceIdentifierSuffixInJs(jsContent, hardwareItemMap.getHardwareItems(HardwareType.BNO055IMU), "AsAdafruitBNO055IMU", "AsBNO055IMU"), "adafruitBNO055IMUParametersAccess", "bno055imuParametersAccess");
    }

    private static String replaceIdentifierSuffixInJs(String jsContent, List<HardwareItem> hardwareItemList, String oldIdentifierSuffix, String newIdentifierSuffix) {
        if (hardwareItemList != null) {
            for (HardwareItem hardwareItem : hardwareItemList) {
                String newIdentifier = hardwareItem.identifier;
                if (newIdentifier.endsWith(newIdentifierSuffix)) {
                    String oldIdentifier = newIdentifier.substring(0, newIdentifier.length() - newIdentifierSuffix.length()) + oldIdentifierSuffix;
                    String oldCode = oldIdentifier + ".";
                    String newCode = newIdentifier + ".";
                    jsContent = jsContent.replace(oldCode, newCode);
                }
            }
        }
        return jsContent;
    }

    private static String replaceIdentifierInJs(String jsContent, String oldIdentifier, String newIdentifier) {
        String oldCode = oldIdentifier + ".";
        String newCode = newIdentifier + ".";
        return jsContent.replace(oldCode, newCode);
    }

    public static String getConfigurationName() {
        RobotConfigFileManager robotConfigFileManager = new RobotConfigFileManager();
        RobotConfigFile activeConfig = robotConfigFileManager.getActiveConfig();
        return activeConfig.getName();
    }

    private static void buildKnownTypesAndReservedWordsForFtcJava() {
        KNOWN_TYPES_FOR_FTCJAVA.put("android.graphics", new Class[]{Color.class});
        KNOWN_TYPES_FOR_FTCJAVA.put("android.util", new Class[]{android.util.Size.class});
        KNOWN_TYPES_FOR_FTCJAVA.put("com.qualcomm.ftccommon", new Class[]{SoundPlayer.class});
        KNOWN_TYPES_FOR_FTCJAVA.put("com.qualcomm.hardware.andymark", new Class[]{AndyMarkIMU.class, AndyMarkIMUOrientationOnRobot.class, AndyMarkIMUOrientationOnRobot.I2cPortFacingDirection.class, AndyMarkIMUOrientationOnRobot.LogoFacingDirection.class, AndyMarkColorSensor.class});
        KNOWN_TYPES_FOR_FTCJAVA.put("com.qualcomm.hardware.bosch", new Class[]{BNO055IMU.class, BNO055IMU.AccelerationIntegrator.class, BNO055IMU.AccelUnit.class, BNO055IMU.Parameters.class, BNO055IMU.SensorMode.class, BNO055IMU.SystemStatus.class, JustLoggingAccelerationIntegrator.class});
        KNOWN_TYPES_FOR_FTCJAVA.put("com.qualcomm.hardware.dfrobot", new Class[]{HuskyLens.class});
        KNOWN_TYPES_FOR_FTCJAVA.put("com.qualcomm.hardware.digitalchickenlabs", new Class[]{OctoQuad.class});
        KNOWN_TYPES_FOR_FTCJAVA.put("com.qualcomm.hardware.gobilda", new Class[]{GoBildaPinpointDriver.class, GoBildaPinpointDriver.DeviceStatus.class, GoBildaPinpointDriver.EncoderDirection.class, GoBildaPinpointDriver.GoBildaOdometryPods.class, GoBildaPinpointDriver.ReadData.class});
        KNOWN_TYPES_FOR_FTCJAVA.put("com.qualcomm.hardware.limelightvision", new Class[]{Limelight3A.class, LLResult.class, LLResultTypes.class, LLResultTypes.FiducialResult.class, LLResultTypes.ColorResult.class, LLStatus.class});
        KNOWN_TYPES_FOR_FTCJAVA.put("com.qualcomm.hardware.maxbotix", new Class[]{MaxSonarI2CXL.class});
        KNOWN_TYPES_FOR_FTCJAVA.put("com.qualcomm.hardware.modernrobotics", new Class[]{ModernRoboticsI2cCompassSensor.class, ModernRoboticsI2cGyro.class, ModernRoboticsI2cGyro.HeadingMode.class, ModernRoboticsI2cRangeSensor.class});
        KNOWN_TYPES_FOR_FTCJAVA.put("com.qualcomm.hardware.rev", new Class[]{RevBlinkinLedDriver.class, RevBlinkinLedDriver.BlinkinPattern.class, RevHubOrientationOnRobot.class, RevHubOrientationOnRobot.LogoFacingDirection.class, RevHubOrientationOnRobot.UsbFacingDirection.class, Rev9AxisImuOrientationOnRobot.class, Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection.class, Rev9AxisImuOrientationOnRobot.LogoFacingDirection.class});
        KNOWN_TYPES_FOR_FTCJAVA.put("com.qualcomm.hardware.sparkfun", new Class[]{SparkFunLEDStick.class, SparkFunOTOS.class, SparkFunOTOS.Pose2D.class, SparkFunOTOS.SelfTestConfig.class, SparkFunOTOS.SignalProcessConfig.class, SparkFunOTOS.Status.class, SparkFunOTOS.Version.class});
        KNOWN_TYPES_FOR_FTCJAVA.put("com.qualcomm.robotcore.eventloop.opmode", new Class[]{Autonomous.class, Disabled.class, LinearOpMode.class, OpMode.class, TeleOp.class});
        KNOWN_TYPES_FOR_FTCJAVA.put("com.qualcomm.robotcore.hardware", new Class[]{AccelerationSensor.class, AnalogInput.class, CRServo.class, ColorSensor.class, CompassSensor.class, CompassSensor.CompassMode.class, DcMotor.class, DcMotor.RunMode.class, DcMotor.ZeroPowerBehavior.class, DcMotorEx.class, DcMotorSimple.class, DcMotorSimple.Direction.class, DigitalChannel.class, DigitalChannel.Mode.class, DistanceSensor.class, Gamepad.class, Gamepad.LedEffect.class, Gamepad.LedEffect.Builder.class, Gamepad.RumbleEffect.class, Gamepad.RumbleEffect.Builder.class, GyroSensor.class, Gyroscope.class, I2cAddr.class, I2cAddrConfig.class, I2cAddressableDevice.class, IMU.class, IMU.Parameters.class, ImuOrientationOnRobot.class, IrSeekerSensor.class, IrSeekerSensor.Mode.class, LED.class, Light.class, LightSensor.class, MotorControlAlgorithm.class, NormalizedColorSensor.class, NormalizedRGBA.class, OpticalDistanceSensor.class, OrientationSensor.class, PIDCoefficients.class, PIDFCoefficients.class, PWMOutput.class, Servo.class, Servo.Direction.class, ServoController.class, ServoController.PwmStatus.class, SwitchableLight.class, TouchSensor.class, UltrasonicSensor.class, VoltageSensor.class});
        KNOWN_TYPES_FOR_FTCJAVA.put("com.qualcomm.robotcore.util", new Class[]{ElapsedTime.class, ElapsedTime.Resolution.class, Range.class, ReadWriteFile.class, RobotLog.class, SortOrder.class});
        KNOWN_TYPES_FOR_FTCJAVA.put("java.lang", new Class[]{Boolean.class, Byte.class, Character.class, Double.class, Float.class, Integer.class, Long.class, Number.class, Object.class, Short.class, String.class});
        KNOWN_TYPES_FOR_FTCJAVA.put("java.util", new Class[]{ArrayList.class, Collections.class, List.class});
        KNOWN_TYPES_FOR_FTCJAVA.put("java.util.concurrent", new Class[]{TimeUnit.class});
        KNOWN_TYPES_FOR_FTCJAVA.put("org.firstinspires.ftc.robotcore.external", new Class[]{ClassFactory.class, JavaUtil.class, Telemetry.class, Telemetry.DisplayFormat.class});
        KNOWN_TYPES_FOR_FTCJAVA.put("org.firstinspires.ftc.robotcore.external.android", new Class[]{AndroidAccelerometer.class, AndroidGyroscope.class, AndroidOrientation.class, AndroidSoundPool.class, AndroidTextToSpeech.class});
        KNOWN_TYPES_FOR_FTCJAVA.put("org.firstinspires.ftc.robotcore.external.hardware.camera", new Class[]{BuiltinCameraDirection.class, CameraName.class, WebcamName.class});
        KNOWN_TYPES_FOR_FTCJAVA.put("org.firstinspires.ftc.robotcore.external.hardware.camera.controls", new Class[]{CameraControl.class, ExposureControl.class, FocusControl.class, GainControl.class, PtzControl.class, WhiteBalanceControl.class});
        KNOWN_TYPES_FOR_FTCJAVA.put("org.firstinspires.ftc.robotcore.external.matrices", new Class[]{MatrixF.class, OpenGLMatrix.class, VectorF.class});
        KNOWN_TYPES_FOR_FTCJAVA.put("org.firstinspires.ftc.robotcore.external.navigation", new Class[]{Acceleration.class, AngleUnit.class, AngularVelocity.class, AxesOrder.class, AxesReference.class, Axis.class, CurrentUnit.class, DistanceUnit.class, MagneticFlux.class, Orientation.class, Pose2D.class, Pose3D.class, Position.class, Quaternion.class, TempUnit.class, Temperature.class, UnnormalizedAngleUnit.class, Velocity.class, YawPitchRollAngles.class});
        KNOWN_TYPES_FOR_FTCJAVA.put("org.firstinspires.ftc.robotcore.external.stream", new Class[]{CameraStreamServer.class});
        KNOWN_TYPES_FOR_FTCJAVA.put("org.firstinspires.ftc.robotcore.internal.system", new Class[]{AppUtil.class});
        KNOWN_TYPES_FOR_FTCJAVA.put("org.firstinspires.ftc.vision", new Class[]{VisionPortal.class, VisionProcessor.class});
        KNOWN_TYPES_FOR_FTCJAVA.put("org.firstinspires.ftc.vision.apriltag", new Class[]{AprilTagDetection.class, AprilTagGameDatabase.class, AprilTagLibrary.class, AprilTagMetadata.class, AprilTagPoseFtc.class, AprilTagPoseRaw.class, AprilTagProcessor.class});
        KNOWN_TYPES_FOR_FTCJAVA.put("org.firstinspires.ftc.vision.opencv", new Class[]{Circle.class, ColorBlobLocatorProcessor.class, ColorRange.class, ColorSpace.class, ImageRegion.class, PredominantColorProcessor.class});
        KNOWN_TYPES_FOR_FTCJAVA.put("org.opencv.core", new Class[]{RotatedRect.class, Scalar.class});
        for (Class<?>[] classes : KNOWN_TYPES_FOR_FTCJAVA.values()) {
            for (Class<?> c : classes) {
                if (c.getEnclosingClass() == null) {
                    RESERVED_WORDS_FOR_FTCJAVA.add(c.getSimpleName());
                }
            }
        }
        for (Method method : LinearOpMode.class.getMethods()) {
            if (!method.getName().equals("runOpMode")) {
                RESERVED_WORDS_FOR_FTCJAVA.add(method.getName());
            }
        }
        for (Field field : LinearOpMode.class.getFields()) {
            RESERVED_WORDS_FOR_FTCJAVA.add(field.getName());
        }
        RESERVED_WORDS_FOR_FTCJAVA.add("abstract");
        RESERVED_WORDS_FOR_FTCJAVA.add("assert");
        RESERVED_WORDS_FOR_FTCJAVA.add("boolean");
        RESERVED_WORDS_FOR_FTCJAVA.add("break");
        RESERVED_WORDS_FOR_FTCJAVA.add("byte");
        RESERVED_WORDS_FOR_FTCJAVA.add("case");
        RESERVED_WORDS_FOR_FTCJAVA.add("catch");
        RESERVED_WORDS_FOR_FTCJAVA.add("char");
        RESERVED_WORDS_FOR_FTCJAVA.add("class");
        RESERVED_WORDS_FOR_FTCJAVA.add("const");
        RESERVED_WORDS_FOR_FTCJAVA.add("continue");
        RESERVED_WORDS_FOR_FTCJAVA.add("default");
        RESERVED_WORDS_FOR_FTCJAVA.add("do");
        RESERVED_WORDS_FOR_FTCJAVA.add("double");
        RESERVED_WORDS_FOR_FTCJAVA.add("else");
        RESERVED_WORDS_FOR_FTCJAVA.add("enum");
        RESERVED_WORDS_FOR_FTCJAVA.add("extends");
        RESERVED_WORDS_FOR_FTCJAVA.add("final");
        RESERVED_WORDS_FOR_FTCJAVA.add("finally");
        RESERVED_WORDS_FOR_FTCJAVA.add("float");
        RESERVED_WORDS_FOR_FTCJAVA.add("for");
        RESERVED_WORDS_FOR_FTCJAVA.add("goto");
        RESERVED_WORDS_FOR_FTCJAVA.add("if");
        RESERVED_WORDS_FOR_FTCJAVA.add("implements");
        RESERVED_WORDS_FOR_FTCJAVA.add("import");
        RESERVED_WORDS_FOR_FTCJAVA.add("instanceof");
        RESERVED_WORDS_FOR_FTCJAVA.add("int");
        RESERVED_WORDS_FOR_FTCJAVA.add("interface");
        RESERVED_WORDS_FOR_FTCJAVA.add("long");
        RESERVED_WORDS_FOR_FTCJAVA.add("native");
        RESERVED_WORDS_FOR_FTCJAVA.add(RequestConditions.REQUEST_KEY_NEW);
        RESERVED_WORDS_FOR_FTCJAVA.add("package");
        RESERVED_WORDS_FOR_FTCJAVA.add("private");
        RESERVED_WORDS_FOR_FTCJAVA.add("protected");
        RESERVED_WORDS_FOR_FTCJAVA.add("public");
        RESERVED_WORDS_FOR_FTCJAVA.add("return");
        RESERVED_WORDS_FOR_FTCJAVA.add("short");
        RESERVED_WORDS_FOR_FTCJAVA.add("static");
        RESERVED_WORDS_FOR_FTCJAVA.add("strictfp");
        RESERVED_WORDS_FOR_FTCJAVA.add("super");
        RESERVED_WORDS_FOR_FTCJAVA.add("switch");
        RESERVED_WORDS_FOR_FTCJAVA.add("synchronized");
        RESERVED_WORDS_FOR_FTCJAVA.add("this");
        RESERVED_WORDS_FOR_FTCJAVA.add("throw");
        RESERVED_WORDS_FOR_FTCJAVA.add("throws");
        RESERVED_WORDS_FOR_FTCJAVA.add("transient");
        RESERVED_WORDS_FOR_FTCJAVA.add("try");
        RESERVED_WORDS_FOR_FTCJAVA.add("void");
        RESERVED_WORDS_FOR_FTCJAVA.add("volatile");
        RESERVED_WORDS_FOR_FTCJAVA.add("while");
        RESERVED_WORDS_FOR_FTCJAVA.add(AbstractMethodError.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(Appendable.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(ArithmeticException.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(ArrayIndexOutOfBoundsException.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(ArrayStoreException.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(AssertionError.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(AutoCloseable.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(Boolean.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add("BootstrapMethodError");
        RESERVED_WORDS_FOR_FTCJAVA.add(Byte.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(Character.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(CharSequence.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(Class.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add("ClassValue");
        RESERVED_WORDS_FOR_FTCJAVA.add(ClassCastException.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(ClassCircularityError.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(ClassFormatError.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(ClassLoader.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(ClassNotFoundException.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(Cloneable.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(CloneNotSupportedException.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(Comparable.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(Compiler.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(Deprecated.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(Double.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(Enum.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(EnumConstantNotPresentException.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(Error.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(Exception.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(ExceptionInInitializerError.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(Float.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(FunctionalInterface.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(IllegalAccessError.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(IllegalAccessException.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(IllegalArgumentException.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(IllegalMonitorStateException.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(IllegalStateException.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(IllegalThreadStateException.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(IncompatibleClassChangeError.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(IndexOutOfBoundsException.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(InheritableThreadLocal.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(InstantiationError.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(InstantiationException.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(Integer.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(InternalError.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(InterruptedException.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(Iterable.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(LinkageError.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(Long.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(Math.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(NegativeArraySizeException.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(NoClassDefFoundError.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(NoSuchFieldError.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(NoSuchFieldException.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(NoSuchMethodError.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(NoSuchMethodException.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(NullPointerException.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(Number.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(NumberFormatException.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(Object.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(OutOfMemoryError.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(Override.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(Package.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(Process.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(ProcessBuilder.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(Readable.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(ReflectiveOperationException.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(Runnable.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(Runtime.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(RuntimeException.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(RuntimePermission.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(SafeVarargs.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(SecurityException.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(SecurityManager.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(Short.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(StackOverflowError.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(StackTraceElement.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(StrictMath.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(String.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(StringBuffer.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(StringBuilder.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(StringIndexOutOfBoundsException.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(SuppressWarnings.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(System.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(Thread.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(ThreadDeath.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(ThreadGroup.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(ThreadLocal.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(Throwable.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(TypeNotPresentException.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(UnknownError.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(UnsatisfiedLinkError.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(UnsupportedClassVersionError.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(UnsupportedOperationException.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(VerifyError.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(VirtualMachineError.class.getSimpleName());
        RESERVED_WORDS_FOR_FTCJAVA.add(Void.class.getSimpleName());
    }
}
