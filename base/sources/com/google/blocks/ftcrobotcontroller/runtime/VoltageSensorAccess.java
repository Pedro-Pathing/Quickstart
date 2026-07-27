package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/* JADX INFO: loaded from: classes8.dex */
class VoltageSensorAccess extends HardwareAccess<VoltageSensor> {
    private final VoltageSensor voltageSensor;

    VoltageSensorAccess(BlocksOpMode blocksOpMode, HardwareItem hardwareItem, HardwareMap hardwareMap) {
        super(blocksOpMode, hardwareItem, hardwareMap, VoltageSensor.class);
        this.voltageSensor = (VoltageSensor) this.hardwareDevice;
    }

    @JavascriptInterface
    @Block(classes = {VoltageSensor.class}, methodName = {"getVoltage"})
    public double getVoltage() {
        try {
            startBlockExecution(BlockType.GETTER, ".Voltage");
            return this.voltageSensor.getVoltage();
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }
}
