package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;

/* JADX INFO: loaded from: classes8.dex */
class LightSensorAccess extends HardwareAccess<LightSensor> {
    private final LightSensor lightSensor;

    LightSensorAccess(BlocksOpMode blocksOpMode, HardwareItem hardwareItem, HardwareMap hardwareMap) {
        super(blocksOpMode, hardwareItem, hardwareMap, LightSensor.class);
        this.lightSensor = (LightSensor) this.hardwareDevice;
    }

    @JavascriptInterface
    @Block(classes = {LightSensor.class}, methodName = {"getLightDetected"})
    public double getLightDetected() {
        try {
            startBlockExecution(BlockType.GETTER, ".LightDetected");
            return this.lightSensor.getLightDetected();
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
    @Block(classes = {LightSensor.class}, methodName = {"getRawLightDetected"})
    public double getRawLightDetected() {
        try {
            startBlockExecution(BlockType.GETTER, ".RawLightDetected");
            return this.lightSensor.getRawLightDetected();
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
    @Block(classes = {LightSensor.class}, methodName = {"getRawLightDetectedMax"})
    public double getRawLightDetectedMax() {
        try {
            startBlockExecution(BlockType.GETTER, ".RawLightDetectedMax");
            return this.lightSensor.getRawLightDetectedMax();
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
    @Block(classes = {LightSensor.class}, methodName = {"enableLed"})
    public void enableLed(boolean enable) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".enableLed");
            this.lightSensor.enableLed(enable);
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
