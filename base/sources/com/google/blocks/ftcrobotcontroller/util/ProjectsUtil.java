package com.google.blocks.ftcrobotcontroller.util;

import android.content.res.AssetManager;
import android.text.Html;
import android.util.Xml;
import com.google.blocks.ftcrobotcontroller.IOExceptionWithUserVisibleMessage;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItemMap;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareType;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareUtil;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.robotcore.util.RobotLog;
import com.sun.tools.doclint.DocLint;
import java.io.File;
import java.io.FilenameFilter;
import java.io.IOException;
import java.io.StringReader;
import java.io.StringWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.firstinspires.ftc.onbotjava.OnBotJavaFileSystemUtils;
import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.ThrowingCallable;
import org.firstinspires.ftc.robotcore.internal.opmode.OnBotJavaHelper;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.xmlpull.v1.XmlPullParser;
import org.xmlpull.v1.XmlPullParserException;
import org.xmlpull.v1.XmlSerializer;

/* JADX INFO: loaded from: classes8.dex */
public class ProjectsUtil {
    private static final String BLOCKS_SAMPLES_PATH = "blocks/samples";
    private static final String DEFAULT_BLOCKS_SAMPLE_NAME = "default";
    public static final String TAG = "ProjectsUtil";
    public static final String VALID_PROJECT_REGEX = "^[a-zA-Z0-9 \\!\\#\\$\\%\\&\\'\\(\\)\\+\\,\\-\\.\\;\\=\\@\\[\\]\\^_\\{\\}\\~]+$";
    private static final String XML_ATTRIBUTE_AUTO_TRANSITION = "autoTransition";
    private static final String XML_ATTRIBUTE_FLAVOR = "flavor";
    private static final String XML_ATTRIBUTE_GROUP = "group";
    private static final String XML_ATTRIBUTE_VALUE = "value";
    public static final String XML_END_TAG = "</xml>";
    public static final String XML_EXTRA_START = "<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>";
    private static final String XML_TAG_ENABLED = "Enabled";
    private static final String XML_TAG_EXTRA = "Extra";
    private static final String XML_TAG_OP_MODE_META = "OpModeMeta";
    private static final OpModeMeta.Flavor DEFAULT_FLAVOR = OpModeMeta.Flavor.TELEOP;
    private static final Pattern identifierFieldPattern = Pattern.compile("<field name=\"(IDENTIFIER|IDENTIFIER1|IDENTIFIER2|WEBCAM_NAME)\">(.*)</field>");
    private static final Pattern deviceNameWithSuffix = Pattern.compile("(.*)(As.*)");
    private static final String[] specialDeviceNames = {"left", "right"};

    private ProjectsUtil() {
    }

    public static String fetchProjectsWithBlocks() {
        AppUtil.getInstance().ensureDirectoryExists(AppUtil.BLOCK_OPMODES_DIR, false);
        ReadWriteFile.ensureAllChangesAreCommitted(AppUtil.BLOCK_OPMODES_DIR);
        return (String) ProjectsLockManager.lockProjectsWhile(new Supplier<String>() { // from class: com.google.blocks.ftcrobotcontroller.util.ProjectsUtil.1
            @Override // org.firstinspires.ftc.robotcore.external.Supplier
            public String get() {
                File[] files = AppUtil.BLOCK_OPMODES_DIR.listFiles(new BlocksProjectFilenameFilter());
                if (files != null) {
                    StringBuilder jsonProjects = new StringBuilder();
                    jsonProjects.append("[");
                    String delimiter = "";
                    for (File file : files) {
                        String filename = file.getName();
                        String projectName = filename.substring(0, filename.length() - AppUtil.BLOCKS_BLK_EXT.length());
                        try {
                            boolean enabled = ProjectsUtil.isProjectEnabled(projectName);
                            jsonProjects.append(delimiter).append("{").append("\"name\":\"").append(ProjectsUtil.escapeDoubleQuotes(projectName)).append("\", ").append("\"escapedName\":\"").append(ProjectsUtil.escapeDoubleQuotes(Html.escapeHtml(projectName))).append("\", ").append("\"dateModifiedMillis\":").append(file.lastModified()).append(", ").append("\"enabled\":").append(enabled).append("}");
                            delimiter = DocLint.TAGS_SEPARATOR;
                        } catch (CorruptFileException e) {
                            RobotLog.ee(ProjectsUtil.TAG, "While fetching projects with blocks, " + projectName + " could not be read, skipping");
                        } catch (IOException e2) {
                            RobotLog.ee(ProjectsUtil.TAG, "fetchProjectsWithBlocks() - problem with project " + projectName);
                            RobotLog.logStackTrace(e2);
                        }
                    }
                    jsonProjects.append("]");
                    return jsonProjects.toString();
                }
                return "[]";
            }
        });
    }

    public static String escapeSingleQuotes(String s) {
        return s.replace("'", "\\'");
    }

    public static String escapeDoubleQuotes(String s) {
        return s.replace("\"", "\\\"");
    }

    /* JADX INFO: Access modifiers changed from: private */
    public static int findExtraXml(String blkFileContent) {
        int i = blkFileContent.indexOf(XML_END_TAG);
        if (i != -1) {
            return XML_END_TAG.length() + i;
        }
        int i2 = blkFileContent.lastIndexOf(XML_EXTRA_START);
        if (i2 == -1) {
            return -1;
        }
        return i2;
    }

    public static void fetchProjectsForOfflineBlocksEditor(final List<OfflineBlocksProject> offlineBlocksProjects) throws Throwable {
        AppUtil.getInstance().ensureDirectoryExists(AppUtil.BLOCK_OPMODES_DIR, false);
        ReadWriteFile.ensureAllChangesAreCommitted(AppUtil.BLOCK_OPMODES_DIR);
        ProjectsLockManager.lockProjectsWhile(new ThrowingCallable<Void, IOException>() { // from class: com.google.blocks.ftcrobotcontroller.util.ProjectsUtil.2
            @Override // org.firstinspires.ftc.robotcore.external.ThrowingCallable
            public Void call() throws IOException {
                File[] files;
                String projectName;
                String blkFileContent;
                boolean isProjectEnabled;
                File[] files2 = AppUtil.BLOCK_OPMODES_DIR.listFiles(new BlocksProjectFilenameFilter());
                if (files2 != null) {
                    int length = files2.length;
                    int i = 0;
                    int i2 = 0;
                    while (i2 < length) {
                        File file = files2[i2];
                        String filename = file.getName();
                        String projectName2 = filename.substring(i, filename.length() - AppUtil.BLOCKS_BLK_EXT.length());
                        try {
                            blkFileContent = ProjectsUtil.fetchBlkFileContent(projectName2);
                            int iExtraXml = ProjectsUtil.findExtraXml(blkFileContent);
                            if (iExtraXml == -1) {
                                RobotLog.ee(ProjectsUtil.TAG, "Block file, " + projectName2 + ", for offline blocks editor is missing extra xml.");
                                isProjectEnabled = true;
                            } else {
                                try {
                                    String extraXml = blkFileContent.substring(iExtraXml);
                                    boolean isProjectEnabled2 = ProjectsUtil.isProjectEnabled(projectName2, extraXml);
                                    isProjectEnabled = isProjectEnabled2;
                                } catch (CorruptFileException e) {
                                    files = files2;
                                    projectName = projectName2;
                                    RobotLog.ee(ProjectsUtil.TAG, "Block file, " + projectName + ", for offline blocks editor could not be read, skipping");
                                    i2++;
                                    files2 = files;
                                    i = 0;
                                }
                            }
                            files = files2;
                            projectName = projectName2;
                        } catch (CorruptFileException e2) {
                            files = files2;
                            projectName = projectName2;
                        }
                        try {
                            offlineBlocksProjects.add(new OfflineBlocksProject(filename, blkFileContent, projectName2, file.lastModified(), isProjectEnabled));
                        } catch (CorruptFileException e3) {
                            RobotLog.ee(ProjectsUtil.TAG, "Block file, " + projectName + ", for offline blocks editor could not be read, skipping");
                        }
                        i2++;
                        files2 = files;
                        i = 0;
                    }
                    return null;
                }
                return null;
            }
        });
    }

    public static void fetchProjects(final List<BlocksProject> blocksProjects) throws Throwable {
        AppUtil.getInstance().ensureDirectoryExists(AppUtil.BLOCK_OPMODES_DIR, false);
        ReadWriteFile.ensureAllChangesAreCommitted(AppUtil.BLOCK_OPMODES_DIR);
        ProjectsLockManager.lockProjectsWhile(new ThrowingCallable<Void, IOException>() { // from class: com.google.blocks.ftcrobotcontroller.util.ProjectsUtil.3
            @Override // org.firstinspires.ftc.robotcore.external.ThrowingCallable
            public Void call() throws IOException {
                File[] files = AppUtil.BLOCK_OPMODES_DIR.listFiles(new BlocksProjectFilenameFilter());
                if (files != null) {
                    for (File file : files) {
                        String filename = file.getName();
                        String projectName = filename.substring(0, filename.length() - AppUtil.BLOCKS_BLK_EXT.length());
                        try {
                            String blkFileContent = ProjectsUtil.fetchBlkFileContent(projectName);
                            blocksProjects.add(new BlocksProject(filename, blkFileContent, file.lastModified()));
                        } catch (CorruptFileException e) {
                            RobotLog.ee(ProjectsUtil.TAG, "While fetching blocks projects, block file " + projectName + " could not be read, skipping");
                        }
                    }
                    return null;
                }
                return null;
            }
        });
    }

    public static String fetchSampleNames() throws IOException {
        HardwareItemMap.newHardwareItemMap();
        StringBuilder jsonSamples = new StringBuilder();
        jsonSamples.append("[");
        AssetManager assetManager = AppUtil.getDefContext().getAssets();
        List<String> sampleFileNames = Arrays.asList(assetManager.list(BLOCKS_SAMPLES_PATH));
        Collections.sort(sampleFileNames);
        if (sampleFileNames != null) {
            String delimiter = "";
            for (String filename : sampleFileNames) {
                if (filename.endsWith(AppUtil.BLOCKS_BLK_EXT)) {
                    String sampleName = filename.substring(0, filename.length() - AppUtil.BLOCKS_BLK_EXT.length());
                    if (!sampleName.equals(DEFAULT_BLOCKS_SAMPLE_NAME)) {
                        String blkFileContent = readSample(sampleName);
                        Set<HardwareUtil.Capability> requestedCapabilities = getRequestedCapabilities(sampleName, blkFileContent);
                        jsonSamples.append(delimiter).append("{").append("\"name\":\"").append(escapeDoubleQuotes(sampleName)).append("\", ").append("\"escapedName\":\"").append(escapeDoubleQuotes(Html.escapeHtml(sampleName))).append("\", ").append("\"requestedCapabilities\":[");
                        String delimiter2 = "";
                        for (HardwareUtil.Capability requestedCapability : requestedCapabilities) {
                            jsonSamples.append(delimiter2).append("\"").append(requestedCapability).append("\"");
                            delimiter2 = DocLint.TAGS_SEPARATOR;
                        }
                        jsonSamples.append("]").append("}");
                        delimiter = DocLint.TAGS_SEPARATOR;
                    }
                }
            }
        }
        jsonSamples.append("]");
        return jsonSamples.toString();
    }

    static Map<String, String> getSamples(HardwareItemMap hardwareItemMap) throws IOException {
        Map<String, String> map = new TreeMap<>();
        AssetManager assetManager = AppUtil.getDefContext().getAssets();
        List<String> sampleFileNames = Arrays.asList(assetManager.list(BLOCKS_SAMPLES_PATH));
        for (String filename : sampleFileNames) {
            if (filename.endsWith(AppUtil.BLOCKS_BLK_EXT)) {
                String sampleName = filename.substring(0, filename.length() - AppUtil.BLOCKS_BLK_EXT.length());
                String blkFileContent = readSampleAndReplaceDeviceNames(sampleName, hardwareItemMap);
                if (sampleName.equals(DEFAULT_BLOCKS_SAMPLE_NAME)) {
                    sampleName = "";
                }
                map.put(sampleName, blkFileContent);
            }
        }
        return map;
    }

    private static Set<HardwareUtil.Capability> getRequestedCapabilities(String sampleName, String blkFileContent) {
        Set<HardwareUtil.Capability> requestedCapabilities = new LinkedHashSet<>();
        if (blkFileContent.contains("navigation_switchableCamera_forAllWebcams")) {
            requestedCapabilities.add(HardwareUtil.Capability.SWITCHABLE_CAMERA);
        }
        if (blkFileContent.contains("navigation_typedEnum_builtinCameraDirection") || blkFileContent.contains("navigation_webcamName")) {
            requestedCapabilities.add(HardwareUtil.Capability.VISION);
        }
        return requestedCapabilities;
    }

    public static List<OpModeMeta> fetchEnabledProjectsWithJavaScript() {
        return (List) ProjectsLockManager.lockProjectsWhile(new Supplier<List<OpModeMeta>>() { // from class: com.google.blocks.ftcrobotcontroller.util.ProjectsUtil.4
            @Override // org.firstinspires.ftc.robotcore.external.Supplier
            public List<OpModeMeta> get() {
                String[] filenames = AppUtil.BLOCK_OPMODES_DIR.list(new FilenameFilter() { // from class: com.google.blocks.ftcrobotcontroller.util.ProjectsUtil.4.1
                    @Override // java.io.FilenameFilter
                    public boolean accept(File dir, String filename) {
                        if (!filename.endsWith(AppUtil.BLOCKS_JS_EXT)) {
                            return false;
                        }
                        String projectName = filename.substring(0, filename.length() - AppUtil.BLOCKS_JS_EXT.length());
                        return ProjectsUtil.isValidProjectName(projectName);
                    }
                });
                List<OpModeMeta> projects = new ArrayList<>();
                if (filenames != null) {
                    for (String filename : filenames) {
                        String projectName = filename.substring(0, filename.length() - AppUtil.BLOCKS_JS_EXT.length());
                        try {
                            OpModeMeta opModeMeta = ProjectsUtil.fetchOpModeMeta(projectName);
                            if (opModeMeta != null) {
                                projects.add(opModeMeta);
                            }
                        } catch (Exception e) {
                            RobotLog.ee(ProjectsUtil.TAG, "While fetching enabled projects with js, block file, " + projectName + ", could not be read, skipping");
                        }
                    }
                }
                return projects;
            }
        });
    }

    /* JADX INFO: Access modifiers changed from: private */
    public static OpModeMeta fetchOpModeMeta(String projectName) {
        String extraXml;
        if (!isValidProjectName(projectName)) {
            throw new IllegalArgumentException();
        }
        try {
            ensureChangesAreCommitted(projectName);
            File blkFile = new File(AppUtil.BLOCK_OPMODES_DIR, projectName + AppUtil.BLOCKS_BLK_EXT);
            String blkFileContent = FileUtil.readFile(blkFile);
            int iExtraXml = findExtraXml(blkFileContent);
            if (iExtraXml != -1) {
                extraXml = blkFileContent.substring(iExtraXml);
                if (!isProjectEnabled(projectName, extraXml)) {
                    return null;
                }
            } else {
                RobotLog.ee(TAG, "Block file, " + projectName + ", is missing extra xml.");
                extraXml = "";
            }
            return createOpModeMeta(projectName, extraXml);
        } catch (IOException e) {
            if (!projectName.startsWith("backup_")) {
                RobotLog.e("ProjectsUtil.fetchOpModeMeta(\"" + projectName + "\") - failed.");
                RobotLog.logStackTrace(e);
            }
            return null;
        }
    }

    public static boolean isValidProjectName(String projectName) {
        if (projectName != null) {
            return projectName.matches(VALID_PROJECT_REGEX);
        }
        return false;
    }

    public static String fetchBlkFileContent(String projectName) throws IOException {
        String blocksContent;
        String extraXml;
        if (!isValidProjectName(projectName)) {
            throw new IllegalArgumentException();
        }
        ensureChangesAreCommitted(projectName);
        File blkFile = new File(AppUtil.BLOCK_OPMODES_DIR, projectName + AppUtil.BLOCKS_BLK_EXT);
        String blkFileContent = FileUtil.readFile(blkFile);
        int iExtraXml = findExtraXml(blkFileContent);
        if (iExtraXml != -1) {
            blocksContent = blkFileContent.substring(0, iExtraXml);
            extraXml = blkFileContent.substring(iExtraXml);
        } else {
            RobotLog.ee(TAG, "Block file, " + projectName + ", is missing extra xml.");
            blocksContent = blkFileContent;
            extraXml = "";
        }
        String upgradedBlocksContent = upgradeBlocks(blocksContent, HardwareItemMap.newHardwareItemMap());
        if (!upgradedBlocksContent.equals(blocksContent)) {
            return upgradedBlocksContent + extraXml;
        }
        return blkFileContent;
    }

    private static String upgradeBlocks(String blkContent, HardwareItemMap hardwareItemMap) {
        String blkContent2 = replaceIdentifierSuffixInBlocks(replaceIdentifierSuffixInBlocks(replaceIdentifierSuffixInBlocks(blkContent.replace("<block type=\"colorBlobLocatorProcessor_clearFilters", "<block type=\"colorBlobLocatorProcessor_removeAllFilters").replace("<block type=\"linearOpMode_resetStartTime", "<block type=\"linearOpMode_resetRuntime").replace("<block type=\"tfodCurrentGame_", "<block type=\"tfod_").replace("<block type=\"tfodCustomModel_activate", "<block type=\"tfod_activate").replace("<block type=\"tfodCustomModel_deactivate", "<block type=\"tfod_deactivate").replace("<block type=\"tfodCustomModel_setClippingMargins", "<block type=\"tfod_setClippingMargins").replace("<block type=\"tfodCustomModel_setZoom", "<block type=\"tfod_setZoom").replace("<block type=\"tfodCustomModel_getRecognitions", "<block type=\"tfod_getRecognitions").replace("<block type=\"tfodCustomModel_setModelFromAsset", "<block type=\"tfodLegacy_setModelFromAsset").replace("<block type=\"tfodCustomModel_setModelFromFile", "<block type=\"tfodLegacy_setModelFromFile").replace("<block type=\"tfodCustomModel_initialize_withIsModelTensorFlow2", "<block type=\"tfodLegacy_initialize_withIsModelTensorFlow2").replace("<block type=\"tfodCustomModel_initialize_withAllArgs", "<block type=\"tfodLegacy_initialize_withAllArgs").replace("<block type=\"adafruitBNO055IMU_", "<block type=\"bno055imu_"), hardwareItemMap.getHardwareItems(HardwareType.BNO055IMU), "AsAdafruitBNO055IMU", "AsBNO055IMU").replace("<block type=\"adafruitBNO055IMUParameters_", "<block type=\"bno055imuParameters_").replace("<shadow type=\"adafruitBNO055IMUParameters_", "<shadow type=\"bno055imuParameters_").replace("<value name=\"ADAFRUIT_BNO055IMU_PARAMETERS\">", "<value name=\"BNO055IMU_PARAMETERS\">"), hardwareItemMap.getHardwareItems(HardwareType.LYNX_MODULE), "asLynxModule", "AsREVModule"), hardwareItemMap.getHardwareItems(HardwareType.COLOR_RANGE_SENSOR), "asLynxI2cColorRangeSensor", "AsREVColorRangeSensor");
        HardwareType[] typesThatDidntHaveSuffix = {HardwareType.ACCELERATION_SENSOR, HardwareType.COLOR_SENSOR, HardwareType.COMPASS_SENSOR, HardwareType.CR_SERVO, HardwareType.DC_MOTOR, HardwareType.DISTANCE_SENSOR, HardwareType.GYRO_SENSOR, HardwareType.IR_SEEKER_SENSOR, HardwareType.LED, HardwareType.LIGHT_SENSOR, HardwareType.SERVO, HardwareType.TOUCH_SENSOR, HardwareType.ULTRASONIC_SENSOR};
        for (HardwareType hardwareType : typesThatDidntHaveSuffix) {
            blkContent2 = replaceIdentifierSuffixInBlocks(blkContent2, hardwareItemMap.getHardwareItems(hardwareType), "", hardwareType.identifierSuffixForJavaScript);
        }
        return blkContent2;
    }

    private static String replaceIdentifierSuffixInBlocks(String blkContent, List<HardwareItem> hardwareItemList, String oldIdentifierSuffix, String newIdentifierSuffix) {
        if (hardwareItemList == null) {
            return blkContent;
        }
        String blkContent2 = blkContent;
        for (HardwareItem hardwareItem : hardwareItemList) {
            String newIdentifier = hardwareItem.identifier;
            if (newIdentifier.endsWith(newIdentifierSuffix)) {
                String oldIdentifier = newIdentifier.substring(0, newIdentifier.length() - newIdentifierSuffix.length()) + oldIdentifierSuffix;
                String[] identifierFieldNames = {"IDENTIFIER", "IDENTIFIER1", "IDENTIFIER2"};
                for (String identifierFieldName : identifierFieldNames) {
                    String oldElement = "<field name=\"" + identifierFieldName + "\">" + oldIdentifier + "</field>";
                    String newElement = "<field name=\"" + identifierFieldName + "\">" + newIdentifier + "</field>";
                    blkContent2 = blkContent2.replace(oldElement, newElement);
                }
            }
        }
        return blkContent2;
    }

    public static String fetchJsFileContent(String projectName) throws IOException {
        if (!isValidProjectName(projectName)) {
            throw new IllegalArgumentException();
        }
        ensureChangesAreCommitted(projectName);
        File jsFile = new File(AppUtil.BLOCK_OPMODES_DIR, projectName + AppUtil.BLOCKS_JS_EXT);
        return FileUtil.readFile(jsFile);
    }

    private static void ensureChangesAreCommitted(String projectName) {
        File blkFile = new File(AppUtil.BLOCK_OPMODES_DIR, projectName + AppUtil.BLOCKS_BLK_EXT);
        File jsFile = new File(AppUtil.BLOCK_OPMODES_DIR, projectName + AppUtil.BLOCKS_JS_EXT);
        ReadWriteFile.ensureChangesAreCommitted(blkFile);
        ReadWriteFile.ensureChangesAreCommitted(jsFile);
    }

    public static String newProject(String projectName, String sampleName) throws IOException {
        if (!isValidProjectName(projectName)) {
            throw new IllegalArgumentException();
        }
        return readSampleAndReplaceDeviceNames(sampleName, HardwareItemMap.newHardwareItemMap());
    }

    private static String readSampleAndReplaceDeviceNames(String sampleName, HardwareItemMap hardwareItemMap) throws IOException {
        return replaceDeviceNamesInSample(readSample(sampleName), hardwareItemMap);
    }

    private static String readSample(String sampleName) throws IOException {
        if (sampleName == null || sampleName.isEmpty()) {
            sampleName = DEFAULT_BLOCKS_SAMPLE_NAME;
        }
        StringBuilder blkFileContent = new StringBuilder();
        AssetManager assetManager = AppUtil.getDefContext().getAssets();
        String assetName = "blocks/samples/" + sampleName + AppUtil.BLOCKS_BLK_EXT;
        FileUtil.readAsset(blkFileContent, assetManager, assetName);
        return blkFileContent.toString();
    }

    private static String replaceDeviceNamesInSample(String blkContent, HardwareItemMap hardwareItemMap) throws IOException {
        Map<HardwareType, Set<String>> sampleDeviceNamesMap = getSampleDeviceNamesMap(blkContent);
        for (Map.Entry<HardwareType, Set<String>> entry : sampleDeviceNamesMap.entrySet()) {
            HardwareType hardwareType = entry.getKey();
            Set<String> sampleDeviceNames = entry.getValue();
            if (!sampleDeviceNames.isEmpty() && hardwareItemMap.contains(hardwareType)) {
                List<HardwareItem> items = hardwareItemMap.getHardwareItems(hardwareType);
                if (!items.isEmpty()) {
                    List<String> actualDeviceNames = new ArrayList<>();
                    for (HardwareItem item : items) {
                        actualDeviceNames.add(item.deviceName);
                    }
                    Map<String, String> replacements = figureOutReplacements(sampleDeviceNames, actualDeviceNames);
                    for (Map.Entry<String, String> replacementEntry : replacements.entrySet()) {
                        String from = replacementEntry.getKey();
                        String to = replacementEntry.getValue();
                        blkContent = replaceDeviceNameInBlocks(hardwareType, blkContent, from, to);
                    }
                }
            }
        }
        return blkContent;
    }

    private static Map<HardwareType, Set<String>> getSampleDeviceNamesMap(String blkFileContent) {
        HardwareType hardwareType;
        Map<String, Set<String>> mapBySuffix = getSampleDeviceNamesMappedBySuffix(blkFileContent);
        Map<HardwareType, Set<String>> sampleDeviceNamesMap = new HashMap<>();
        for (Map.Entry<String, Set<String>> entry : mapBySuffix.entrySet()) {
            String suffix = entry.getKey();
            if (suffix == "WEBCAM_NAME") {
                hardwareType = HardwareType.WEBCAM_NAME;
            } else {
                hardwareType = HardwareType.fromIdentifierSuffixForJavaScript(suffix);
            }
            if (hardwareType != null) {
                sampleDeviceNamesMap.put(hardwareType, entry.getValue());
            }
        }
        return sampleDeviceNamesMap;
    }

    static Map<String, Set<String>> getSampleDeviceNamesMappedBySuffix(String blkFileContent) {
        String deviceName;
        String suffix;
        Map<String, Set<String>> mapBySuffix = new HashMap<>();
        for (String line : blkFileContent.split("\n")) {
            Matcher matcher1 = identifierFieldPattern.matcher(line);
            if (matcher1.find()) {
                String fieldName = matcher1.group(1);
                String fieldValue = matcher1.group(2);
                if (!fieldValue.equals("gamepad1") && !fieldValue.equals("gamepad2")) {
                    if (fieldName.equals("WEBCAM_NAME")) {
                        deviceName = fieldValue;
                        suffix = "WEBCAM_NAME";
                    } else {
                        Matcher matcher2 = deviceNameWithSuffix.matcher(fieldValue);
                        if (matcher2.find()) {
                            deviceName = matcher2.group(1);
                            suffix = matcher2.group(2);
                        }
                    }
                    Set<String> deviceNames = mapBySuffix.get(suffix);
                    if (deviceNames == null) {
                        deviceNames = new HashSet<>();
                        mapBySuffix.put(suffix, deviceNames);
                    }
                    deviceNames.add(deviceName);
                }
            }
        }
        return mapBySuffix;
    }

    static Map<String, String> figureOutReplacements(Collection<String> sampleDeviceNamesArg, Collection<String> actualDeviceNamesArg) {
        Map<String, String> replacements = new HashMap<>();
        List<String> sampleDeviceNames = new ArrayList<>(sampleDeviceNamesArg);
        List<String> actualDeviceNames = new ArrayList<>(actualDeviceNamesArg);
        Collections.sort(sampleDeviceNames);
        Collections.sort(actualDeviceNames);
        Iterator<String> itSample = sampleDeviceNames.iterator();
        while (itSample.hasNext()) {
            String sampleDeviceName = itSample.next();
            Iterator<String> itActual = actualDeviceNames.iterator();
            while (true) {
                if (itActual.hasNext()) {
                    String actualDeviceName = itActual.next();
                    if (sampleDeviceName.equalsIgnoreCase(actualDeviceName)) {
                        itSample.remove();
                        itActual.remove();
                        if (!sampleDeviceName.equals(actualDeviceName)) {
                            replacements.put(sampleDeviceName, actualDeviceName);
                        }
                    }
                }
            }
        }
        for (String specialDeviceName : specialDeviceNames) {
            Iterator<String> itSample2 = sampleDeviceNames.iterator();
            while (true) {
                if (itSample2.hasNext()) {
                    String sampleDeviceName2 = itSample2.next();
                    if (sampleDeviceName2.toLowerCase(Locale.ENGLISH).contains(specialDeviceName)) {
                        Iterator<String> itActual2 = actualDeviceNames.iterator();
                        while (true) {
                            if (itActual2.hasNext()) {
                                String actualDeviceName2 = itActual2.next();
                                if (actualDeviceName2.toLowerCase(Locale.ENGLISH).contains(specialDeviceName)) {
                                    itSample2.remove();
                                    itActual2.remove();
                                    replacements.put(sampleDeviceName2, actualDeviceName2);
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }
        while (!sampleDeviceNames.isEmpty() && !actualDeviceNames.isEmpty()) {
            replacements.put(sampleDeviceNames.remove(0), actualDeviceNames.remove(0));
        }
        return replacements;
    }

    private static String replaceDeviceNameInBlocks(HardwareType hardwareType, String blkContent, String sampleDeviceName, String actualDeviceName) {
        String oldIdentifier;
        String newIdentifier;
        String[] fieldNames;
        if (hardwareType == HardwareType.WEBCAM_NAME) {
            oldIdentifier = sampleDeviceName;
            newIdentifier = actualDeviceName;
            fieldNames = new String[]{"WEBCAM_NAME"};
        } else {
            oldIdentifier = hardwareType.makeIdentifier(sampleDeviceName);
            newIdentifier = hardwareType.makeIdentifier(actualDeviceName);
            if (hardwareType == HardwareType.DC_MOTOR) {
                fieldNames = new String[]{"IDENTIFIER", "IDENTIFIER1", "IDENTIFIER2"};
            } else {
                fieldNames = new String[]{"IDENTIFIER"};
            }
        }
        for (String fieldName : fieldNames) {
            String oldElement = "<field name=\"" + fieldName + "\">" + oldIdentifier + "</field>";
            String newElement = "<field name=\"" + fieldName + "\">" + newIdentifier + "</field>";
            String blkContent2 = blkContent.replace(oldElement, newElement);
            String oldElement2 = "\"" + fieldName + "\":\"" + sampleDeviceName + "\"";
            String newElement2 = "\"" + fieldName + "\":\"" + actualDeviceName + "\"";
            blkContent = blkContent2.replace(oldElement2, newElement2);
        }
        return blkContent;
    }

    public static void saveProject(final String projectName, final String blkFileContent, final String jsFileContent) throws Throwable {
        if (!isValidProjectName(projectName)) {
            throw new IllegalArgumentException();
        }
        ProjectsLockManager.lockProjectsWhile(new ThrowingCallable<Void, IOException>() { // from class: com.google.blocks.ftcrobotcontroller.util.ProjectsUtil.5
            @Override // org.firstinspires.ftc.robotcore.external.ThrowingCallable
            public Void call() throws IOException {
                AppUtil.getInstance().ensureDirectoryExists(AppUtil.BLOCK_OPMODES_DIR, false);
                File blkFile = new File(AppUtil.BLOCK_OPMODES_DIR, projectName + AppUtil.BLOCKS_BLK_EXT);
                File jsFile = new File(AppUtil.BLOCK_OPMODES_DIR, projectName + AppUtil.BLOCKS_JS_EXT);
                ReadWriteFile.updateFileRequiringCommit(blkFile, blkFileContent);
                ReadWriteFile.updateFileRequiringCommit(jsFile, jsFileContent);
                return null;
            }
        });
    }

    public static void renameProject(final String oldProjectName, final String newProjectName) throws Throwable {
        if (!isValidProjectName(oldProjectName) || !isValidProjectName(newProjectName)) {
            throw new IllegalArgumentException();
        }
        ProjectsLockManager.lockProjectsWhile(new ThrowingCallable<Void, IOException>() { // from class: com.google.blocks.ftcrobotcontroller.util.ProjectsUtil.6
            @Override // org.firstinspires.ftc.robotcore.external.ThrowingCallable
            public Void call() throws IOException {
                AppUtil.getInstance().ensureDirectoryExists(AppUtil.BLOCK_OPMODES_DIR, false);
                File oldBlk = new File(AppUtil.BLOCK_OPMODES_DIR, oldProjectName + AppUtil.BLOCKS_BLK_EXT);
                File newBlk = new File(AppUtil.BLOCK_OPMODES_DIR, newProjectName + AppUtil.BLOCKS_BLK_EXT);
                if (oldBlk.renameTo(newBlk)) {
                    File oldJs = new File(AppUtil.BLOCK_OPMODES_DIR, oldProjectName + AppUtil.BLOCKS_JS_EXT);
                    File newJs = new File(AppUtil.BLOCK_OPMODES_DIR, newProjectName + AppUtil.BLOCKS_JS_EXT);
                    oldJs.renameTo(newJs);
                    return null;
                }
                return null;
            }
        });
    }

    public static void copyProject(final String oldProjectName, final String newProjectName) throws Throwable {
        if (!isValidProjectName(oldProjectName) || !isValidProjectName(newProjectName)) {
            throw new IllegalArgumentException();
        }
        ProjectsLockManager.lockProjectsWhile(new ThrowingCallable<Void, IOException>() { // from class: com.google.blocks.ftcrobotcontroller.util.ProjectsUtil.7
            @Override // org.firstinspires.ftc.robotcore.external.ThrowingCallable
            public Void call() throws IOException {
                AppUtil.getInstance().ensureDirectoryExists(AppUtil.BLOCK_OPMODES_DIR, false);
                File oldBlk = new File(AppUtil.BLOCK_OPMODES_DIR, oldProjectName + AppUtil.BLOCKS_BLK_EXT);
                File newBlk = new File(AppUtil.BLOCK_OPMODES_DIR, newProjectName + AppUtil.BLOCKS_BLK_EXT);
                FileUtil.copyFile(oldBlk, newBlk);
                try {
                    File oldJs = new File(AppUtil.BLOCK_OPMODES_DIR, oldProjectName + AppUtil.BLOCKS_JS_EXT);
                    File newJs = new File(AppUtil.BLOCK_OPMODES_DIR, newProjectName + AppUtil.BLOCKS_JS_EXT);
                    FileUtil.copyFile(oldJs, newJs);
                    return null;
                } catch (IOException e) {
                    throw new IOExceptionWithUserVisibleMessage("The Blocks OpMode was successfully copied, but the new OpMode cannot be run until it is saved in the Blocks editor.");
                }
            }
        });
    }

    public static void enableProject(final String projectName, final boolean enable) throws Throwable {
        if (!isValidProjectName(projectName)) {
            throw new IllegalArgumentException();
        }
        ensureChangesAreCommitted(projectName);
        ProjectsLockManager.lockProjectsWhile(new ThrowingCallable<Void, IOException>() { // from class: com.google.blocks.ftcrobotcontroller.util.ProjectsUtil.8
            @Override // org.firstinspires.ftc.robotcore.external.ThrowingCallable
            public Void call() throws IOException {
                String blocksContent;
                String extraXml;
                File blkFile = new File(AppUtil.BLOCK_OPMODES_DIR, projectName + AppUtil.BLOCKS_BLK_EXT);
                String blkFileContent = FileUtil.readFile(blkFile);
                int iExtraXml = ProjectsUtil.findExtraXml(blkFileContent);
                if (iExtraXml != -1) {
                    blocksContent = blkFileContent.substring(0, iExtraXml);
                    extraXml = blkFileContent.substring(iExtraXml);
                } else {
                    RobotLog.ee(ProjectsUtil.TAG, "Block file, " + projectName + ", is missing extra xml.");
                    blocksContent = blkFileContent;
                    extraXml = "";
                }
                OpModeMeta opModeMeta = ProjectsUtil.createOpModeMeta(projectName, extraXml);
                String newBlkFileContent = blocksContent + ProjectsUtil.formatExtraXml(opModeMeta.flavor, opModeMeta.group, opModeMeta.autoTransition, enable);
                ReadWriteFile.updateFileRequiringCommit(blkFile, newBlkFileContent);
                return null;
            }
        });
    }

    public static Boolean deleteProjects(final String[] projectNames) {
        return (Boolean) ProjectsLockManager.lockProjectsWhile(new Supplier<Boolean>() { // from class: com.google.blocks.ftcrobotcontroller.util.ProjectsUtil.9
            /* JADX WARN: Can't rename method to resolve collision */
            @Override // org.firstinspires.ftc.robotcore.external.Supplier
            public Boolean get() {
                for (String str : projectNames) {
                    if (!ProjectsUtil.isValidProjectName(str)) {
                        throw new IllegalArgumentException();
                    }
                }
                boolean success = true;
                for (String projectName : projectNames) {
                    File jsFile = new File(AppUtil.BLOCK_OPMODES_DIR, projectName + AppUtil.BLOCKS_JS_EXT);
                    if (jsFile.exists() && !jsFile.delete()) {
                        success = false;
                    }
                    if (success) {
                        File blkFile = new File(AppUtil.BLOCK_OPMODES_DIR, projectName + AppUtil.BLOCKS_BLK_EXT);
                        if (blkFile.exists() && !blkFile.delete()) {
                            success = false;
                        }
                    }
                }
                return Boolean.valueOf(success);
            }
        });
    }

    public static String getBlocksJavaClassName(String projectName) {
        File file;
        StringBuilder className = new StringBuilder();
        char ch = projectName.charAt(0);
        if (Character.isJavaIdentifierStart(ch)) {
            className.append(ch);
        } else if (Character.isJavaIdentifierPart(ch)) {
            className.append('_').append(ch);
        }
        int length = projectName.length();
        for (int i = 1; i < length; i++) {
            char ch2 = projectName.charAt(i);
            if (Character.isJavaIdentifierPart(ch2)) {
                className.append(ch2);
            }
        }
        File dir = new File(OnBotJavaHelper.srcDir, "org/firstinspires/ftc/teamcode");
        String base = className.toString();
        File file2 = new File(dir, base + OnBotJavaFileSystemUtils.EXT_JAVA_FILE);
        if (file2.exists()) {
            int i2 = 1;
            do {
                i2++;
                file = new File(dir, base + i2 + OnBotJavaFileSystemUtils.EXT_JAVA_FILE);
            } while (file.exists());
            className.append(i2);
        }
        return className.toString();
    }

    public static void saveBlocksJava(final String relativeFileName, final String javaContent) throws Throwable {
        ProjectsLockManager.lockProjectsWhile(new ThrowingCallable<Void, IOException>() { // from class: com.google.blocks.ftcrobotcontroller.util.ProjectsUtil.10
            @Override // org.firstinspires.ftc.robotcore.external.ThrowingCallable
            public Void call() throws IOException {
                AppUtil.getInstance().ensureDirectoryExists(AppUtil.BLOCK_OPMODES_DIR, false);
                int lastSlash = relativeFileName.lastIndexOf(OnBotJavaFileSystemUtils.PATH_SEPARATOR);
                String relativeDir = relativeFileName.substring(0, lastSlash + 1);
                String filename = relativeFileName.substring(lastSlash + 1);
                File dir = new File(AppUtil.BLOCK_OPMODES_DIR, "../java/src/" + relativeDir);
                dir.mkdirs();
                File javaFile = new File(dir, filename);
                ReadWriteFile.updateFileRequiringCommit(javaFile, javaContent);
                return null;
            }
        });
    }

    /* JADX INFO: Access modifiers changed from: private */
    public static String formatExtraXml(OpModeMeta.Flavor flavor, String group, String autoTransition, boolean enabled) throws IOException {
        XmlSerializer serializer = Xml.newSerializer();
        StringWriter writer = new StringWriter();
        serializer.setOutput(writer);
        serializer.startDocument("UTF-8", true);
        serializer.startTag("", XML_TAG_EXTRA);
        serializer.startTag("", XML_TAG_OP_MODE_META);
        serializer.attribute("", XML_ATTRIBUTE_FLAVOR, flavor.toString());
        serializer.attribute("", XML_ATTRIBUTE_GROUP, group);
        if (autoTransition != null) {
            serializer.attribute("", XML_ATTRIBUTE_AUTO_TRANSITION, autoTransition);
        }
        serializer.endTag("", XML_TAG_OP_MODE_META);
        serializer.startTag("", XML_TAG_ENABLED);
        serializer.attribute("", XML_ATTRIBUTE_VALUE, Boolean.toString(enabled));
        serializer.endTag("", XML_TAG_ENABLED);
        serializer.endTag("", XML_TAG_EXTRA);
        serializer.endDocument();
        return writer.toString();
    }

    /* JADX INFO: Access modifiers changed from: private */
    public static OpModeMeta createOpModeMeta(String projectName, String extraXml) {
        OpModeMeta.Flavor flavor = DEFAULT_FLAVOR;
        String group = "";
        String autoTransition = null;
        try {
            XmlPullParser parser = Xml.newPullParser();
            parser.setInput(new StringReader(removeNewLines(extraXml)));
            int eventType = parser.getEventType();
            while (eventType != 1) {
                if (eventType == 2) {
                    if (parser.getName().equals(XML_TAG_OP_MODE_META)) {
                        for (int i = 0; i < parser.getAttributeCount(); i++) {
                            String name = parser.getAttributeName(i);
                            String value = parser.getAttributeValue(i);
                            if (name.equals(XML_ATTRIBUTE_FLAVOR)) {
                                flavor = OpModeMeta.Flavor.valueOf(value.toUpperCase(Locale.ENGLISH));
                            } else if (name.equals(XML_ATTRIBUTE_GROUP)) {
                                if (!value.isEmpty() && !value.equals(OpModeMeta.DefaultGroup)) {
                                    group = value;
                                }
                            } else if (name.equals(XML_ATTRIBUTE_AUTO_TRANSITION) && !value.isEmpty()) {
                                autoTransition = value;
                            }
                        }
                    }
                }
                int i2 = parser.next();
                eventType = i2;
            }
        } catch (IOException | XmlPullParserException e) {
            RobotLog.e("ProjectsUtil.createOpmodeMeta(\"" + projectName + "\", ...) - failed to parse xml.");
            RobotLog.logStackTrace(e);
        }
        OpModeMeta.Builder builder = new OpModeMeta.Builder().setName(projectName).setFlavor(flavor).setGroup(group).setSource(OpModeMeta.Source.BLOCKLY);
        if (autoTransition != null) {
            builder.setTransitionTarget(autoTransition);
        }
        return builder.build();
    }

    /* JADX INFO: Access modifiers changed from: private */
    public static boolean isProjectEnabled(String projectName) throws IOException {
        String extraXml;
        if (!isValidProjectName(projectName)) {
            throw new IllegalArgumentException();
        }
        ensureChangesAreCommitted(projectName);
        File blkFile = new File(AppUtil.BLOCK_OPMODES_DIR, projectName + AppUtil.BLOCKS_BLK_EXT);
        String blkFileContent = FileUtil.readFile(blkFile);
        int iExtraXml = findExtraXml(blkFileContent);
        if (iExtraXml != -1) {
            extraXml = blkFileContent.substring(iExtraXml);
        } else {
            RobotLog.ee(TAG, "Block file, " + projectName + ", is missing extra xml.");
            extraXml = "";
        }
        return isProjectEnabled(projectName, extraXml);
    }

    /* JADX INFO: Access modifiers changed from: private */
    public static boolean isProjectEnabled(String projectName, String extraXml) {
        boolean enabled = true;
        try {
            XmlPullParser parser = Xml.newPullParser();
            parser.setInput(new StringReader(removeNewLines(extraXml)));
            int eventType = parser.getEventType();
            while (eventType != 1) {
                if (eventType == 2) {
                    if (parser.getName().equals(XML_TAG_ENABLED)) {
                        for (int i = 0; i < parser.getAttributeCount(); i++) {
                            String name = parser.getAttributeName(i);
                            String value = parser.getAttributeValue(i);
                            if (name.equals(XML_ATTRIBUTE_VALUE)) {
                                enabled = Boolean.parseBoolean(value);
                            }
                        }
                    }
                }
                int i2 = parser.next();
                eventType = i2;
            }
        } catch (IOException | XmlPullParserException e) {
            RobotLog.e("ProjectsUtil.isProjectEnabled(\"" + projectName + "\", ...) - failed to parse xml.");
            RobotLog.logStackTrace(e);
        }
        return enabled;
    }

    private static String removeNewLines(String text) {
        return text.replace("\n", "");
    }

    static class BlocksProjectFilenameFilter implements FilenameFilter {
        BlocksProjectFilenameFilter() {
        }

        @Override // java.io.FilenameFilter
        public boolean accept(File dir, String filename) {
            if (!filename.endsWith(AppUtil.BLOCKS_BLK_EXT)) {
                return false;
            }
            String projectName = filename.substring(0, filename.length() - AppUtil.BLOCKS_BLK_EXT.length());
            return ProjectsUtil.isValidProjectName(projectName);
        }
    }
}
