package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/* JADX INFO: loaded from: classes8.dex */
class OpticalDistanceSensorAccess extends HardwareAccess<OpticalDistanceSensor> {
    private final OpticalDistanceSensor opticalDistanceSensor;

    OpticalDistanceSensorAccess(BlocksOpMode blocksOpMode, HardwareItem hardwareItem, HardwareMap hardwareMap) {
        super(blocksOpMode, hardwareItem, hardwareMap, OpticalDistanceSensor.class);
        this.opticalDistanceSensor = (OpticalDistanceSensor) this.hardwareDevice;
    }

    @JavascriptInterface
    @Block(classes = {ModernRoboticsAnalogOpticalDistanceSensor.class, OpticalDistanceSensor.class}, methodName = {"getLightDetected"})
    public double getLightDetected() {
        try {
            startBlockExecution(BlockType.GETTER, ".LightDetected");
            return this.opticalDistanceSensor.getLightDetected();
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
    @Block(classes = {ModernRoboticsAnalogOpticalDistanceSensor.class, OpticalDistanceSensor.class}, methodName = {"getRawLightDetected"})
    public double getRawLightDetected() {
        try {
            startBlockExecution(BlockType.GETTER, ".RawLightDetected");
            return this.opticalDistanceSensor.getRawLightDetected();
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
    @Block(classes = {ModernRoboticsAnalogOpticalDistanceSensor.class, OpticalDistanceSensor.class}, methodName = {"getRawLightDetectedMax"})
    public double getRawLightDetectedMax() {
        try {
            startBlockExecution(BlockType.GETTER, ".RawLightDetectedMax");
            return this.opticalDistanceSensor.getRawLightDetectedMax();
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
    @Block(classes = {ModernRoboticsAnalogOpticalDistanceSensor.class}, methodName = {"enableLed"})
    public void enableLed(boolean enable) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".enableLed");
            this.opticalDistanceSensor.enableLed(enable);
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
