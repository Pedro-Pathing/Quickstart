package com.google.blocks.ftcrobotcontroller.hardware;

import com.qualcomm.ftccommon.configuration.RobotConfigFile;
import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;
import com.qualcomm.hardware.andymark.AndyMarkColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.ControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.DeviceConfiguration;
import com.qualcomm.robotcore.hardware.configuration.LynxModuleConfiguration;
import com.qualcomm.robotcore.hardware.configuration.ReadXMLFileHandler;
import com.qualcomm.robotcore.hardware.configuration.ServoHubConfiguration;
import com.qualcomm.robotcore.util.RobotLog;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.SortedMap;
import java.util.SortedSet;
import java.util.TreeMap;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.xmlpull.v1.XmlPullParser;

/* JADX INFO: loaded from: classes8.dex */
public class HardwareItemMap {
    private final SortedMap<HardwareType, List<HardwareItem>> map = new TreeMap();
    private final Set<DeviceConfiguration> devices = new HashSet();

    public static HardwareItemMap newHardwareItemMap() {
        HardwareMap hardwareMap;
        OpModeManagerImpl opModeManagerImpl = OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().getRootActivity());
        if (opModeManagerImpl != null && (hardwareMap = opModeManagerImpl.getHardwareMap()) != null) {
            return newHardwareItemMap(hardwareMap);
        }
        try {
            RobotConfigFileManager robotConfigFileManager = new RobotConfigFileManager();
            RobotConfigFile activeConfig = robotConfigFileManager.getActiveConfig();
            XmlPullParser pullParser = activeConfig.getXml();
            return new HardwareItemMap(pullParser);
        } catch (Exception e) {
            RobotLog.logStackTrace(e);
            return new HardwareItemMap();
        }
    }

    public static HardwareItemMap newHardwareItemMap(HardwareMap hardwareMap) {
        return new HardwareItemMap(hardwareMap);
    }

    HardwareItemMap() {
    }

    private HardwareItemMap(XmlPullParser pullParser) {
        try {
            ReadXMLFileHandler readXMLFileHandler = new ReadXMLFileHandler();
            for (ControllerConfiguration controllerConfiguration : readXMLFileHandler.parse(pullParser)) {
                addDevice(controllerConfiguration);
            }
        } catch (RobotCoreException e) {
            RobotLog.logStackTrace(e);
        }
    }

    private HardwareItemMap(HardwareMap hardwareMap) {
        for (HardwareType hardwareType : HardwareType.values()) {
            SortedSet<String> deviceNames = hardwareMap.getAllNames(hardwareType.deviceType);
            for (String deviceName : deviceNames) {
                HardwareDevice device = hardwareMap.get(deviceName);
                if (!(device instanceof AndyMarkColorSensor) || hardwareType != HardwareType.COLOR_RANGE_SENSOR) {
                    addHardwareItem(hardwareType, deviceName);
                }
            }
        }
    }

    private void addController(ControllerConfiguration<? extends DeviceConfiguration> controllerConfiguration) {
        for (ITEM_T deviceConfiguration : controllerConfiguration.getDevices()) {
            addDevice(deviceConfiguration);
        }
        if (controllerConfiguration instanceof LynxModuleConfiguration) {
            LynxModuleConfiguration lynxModuleConfiguration = (LynxModuleConfiguration) controllerConfiguration;
            for (DeviceConfiguration deviceConfiguration2 : lynxModuleConfiguration.getServos()) {
                addDevice(deviceConfiguration2);
            }
            for (DeviceConfiguration deviceConfiguration3 : lynxModuleConfiguration.getMotors()) {
                addDevice(deviceConfiguration3);
            }
            for (DeviceConfiguration deviceConfiguration4 : lynxModuleConfiguration.getAnalogInputs()) {
                addDevice(deviceConfiguration4);
            }
            for (DeviceConfiguration deviceConfiguration5 : lynxModuleConfiguration.getPwmOutputs()) {
                addDevice(deviceConfiguration5);
            }
            for (DeviceConfiguration deviceConfiguration6 : lynxModuleConfiguration.getI2cDevices()) {
                addDevice(deviceConfiguration6);
            }
            for (DeviceConfiguration deviceConfiguration7 : lynxModuleConfiguration.getDigitalDevices()) {
                addDevice(deviceConfiguration7);
            }
            return;
        }
        if (controllerConfiguration instanceof ServoHubConfiguration) {
            ServoHubConfiguration servoHubConfiguration = (ServoHubConfiguration) controllerConfiguration;
            for (DeviceConfiguration deviceConfiguration8 : servoHubConfiguration.getServos()) {
                addDevice(deviceConfiguration8);
            }
        }
    }

    private void addDevice(DeviceConfiguration deviceConfiguration) {
        if (this.devices.add(deviceConfiguration) && deviceConfiguration.isEnabled()) {
            for (HardwareType hardwareType : HardwareUtil.getHardwareTypes(deviceConfiguration)) {
                addHardwareItem(hardwareType, deviceConfiguration.getName());
            }
            if (deviceConfiguration instanceof ControllerConfiguration) {
                addController((ControllerConfiguration) deviceConfiguration);
            }
        }
    }

    private void addHardwareItem(HardwareType hardwareType, String deviceName) {
        if (deviceName.isEmpty()) {
            RobotLog.w("Blocks cannot support a hardware device (" + hardwareType.deviceType.getSimpleName() + ") whose name is empty.");
            return;
        }
        List<HardwareItem> hardwareItemList = this.map.get(hardwareType);
        if (hardwareItemList == null) {
            hardwareItemList = new ArrayList();
            this.map.put(hardwareType, hardwareItemList);
        }
        for (HardwareItem item : hardwareItemList) {
            if (item.deviceName.equals(deviceName)) {
                return;
            }
        }
        HardwareItem hardwareItem = new HardwareItem(hardwareType, deviceName);
        hardwareItemList.add(hardwareItem);
    }

    public int getHardwareTypeCount() {
        return this.map.size();
    }

    public boolean contains(HardwareType hardwareType) {
        return this.map.containsKey(hardwareType);
    }

    public List<HardwareItem> getHardwareItems(HardwareType hardwareType) {
        List<HardwareItem> list = new ArrayList<>();
        if (this.map.containsKey(hardwareType)) {
            for (HardwareItem hardwareItem : this.map.get(hardwareType)) {
                list.add(hardwareItem);
            }
        }
        Collections.sort(list, new Comparator<HardwareItem>() { // from class: com.google.blocks.ftcrobotcontroller.hardware.HardwareItemMap.1
            @Override // java.util.Comparator
            public int compare(HardwareItem a, HardwareItem b) {
                return a.deviceName.compareTo(b.deviceName);
            }
        });
        return Collections.unmodifiableList(list);
    }

    public Iterable<HardwareItem> getAllHardwareItems() {
        List<HardwareItem> list = new ArrayList<>();
        for (List<HardwareItem> hardwareItems : this.map.values()) {
            list.addAll(hardwareItems);
        }
        Collections.sort(list, new Comparator<HardwareItem>() { // from class: com.google.blocks.ftcrobotcontroller.hardware.HardwareItemMap.2
            @Override // java.util.Comparator
            public int compare(HardwareItem a, HardwareItem b) {
                return a.identifier.compareTo(b.identifier);
            }
        });
        return Collections.unmodifiableList(list);
    }

    public Set<HardwareType> getHardwareTypes() {
        return Collections.unmodifiableSet(this.map.keySet());
    }

    public boolean equals(Object o) {
        if (o instanceof HardwareItemMap) {
            HardwareItemMap that = (HardwareItemMap) o;
            return this.map.equals(that.map);
        }
        return false;
    }

    public int hashCode() {
        return this.map.hashCode();
    }
}
