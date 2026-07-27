package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

/* JADX INFO: loaded from: classes8.dex */
class AnalogInputAccess extends HardwareAccess<AnalogInput> {
    private final AnalogInput analogInput;

    AnalogInputAccess(BlocksOpMode blocksOpMode, HardwareItem hardwareItem, HardwareMap hardwareMap) {
        super(blocksOpMode, hardwareItem, hardwareMap, AnalogInput.class);
        this.analogInput = (AnalogInput) this.hardwareDevice;
    }

    @JavascriptInterface
    @Block(classes = {AnalogInput.class}, methodName = {"getVoltage"})
    public double getVoltage() {
        try {
            startBlockExecution(BlockType.GETTER, ".Voltage");
            return this.analogInput.getVoltage();
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {AnalogInput.class}, methodName = {"getMaxVoltage"})
    public double getMaxVoltage() {
        try {
            startBlockExecution(BlockType.GETTER, ".MaxVoltage");
            return this.analogInput.getMaxVoltage();
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
