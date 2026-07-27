package com.google.blocks.ftcrobotcontroller.hardware;

/* JADX INFO: loaded from: classes8.dex */
public class HardwareItem {
    public final String deviceName;
    public final HardwareType hardwareType;
    public final String identifier;
    public final String visibleName;

    public HardwareItem(HardwareType hardwareType, String deviceName) {
        if (hardwareType == null || deviceName == null) {
            throw new NullPointerException();
        }
        this.hardwareType = hardwareType;
        this.deviceName = deviceName;
        this.identifier = hardwareType.makeIdentifier(deviceName);
        this.visibleName = HardwareUtil.makeVisibleNameForDropdownItem(deviceName);
    }

    static String makeIdentifier(String deviceName) {
        int length = deviceName.length();
        StringBuilder identifier = new StringBuilder();
        char ch = deviceName.charAt(0);
        if (Character.isJavaIdentifierStart(ch)) {
            identifier.append(ch);
        } else if (Character.isJavaIdentifierPart(ch)) {
            identifier.append('_').append(ch);
        }
        for (int i = 1; i < length; i++) {
            char ch2 = deviceName.charAt(i);
            if (Character.isJavaIdentifierPart(ch2)) {
                identifier.append(ch2);
            }
        }
        return identifier.toString();
    }

    public boolean equals(Object o) {
        if (!(o instanceof HardwareItem)) {
            return false;
        }
        HardwareItem that = (HardwareItem) o;
        return this.hardwareType.equals(that.hardwareType) && this.deviceName.equals(that.deviceName) && this.identifier.equals(that.identifier) && this.visibleName.equals(that.visibleName);
    }

    public int hashCode() {
        return this.hardwareType.hashCode() + this.deviceName.hashCode() + this.identifier.hashCode() + this.visibleName.hashCode();
    }
}
