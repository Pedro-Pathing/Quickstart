package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/* JADX INFO: loaded from: classes8.dex */
class CompassSensorAccess extends HardwareAccess<CompassSensor> {
    private final CompassSensor compassSensor;

    CompassSensorAccess(BlocksOpMode blocksOpMode, HardwareItem hardwareItem, HardwareMap hardwareMap) {
        super(blocksOpMode, hardwareItem, hardwareMap, CompassSensor.class);
        this.compassSensor = (CompassSensor) this.hardwareDevice;
    }

    @JavascriptInterface
    @Block(classes = {CompassSensor.class}, methodName = {"getDirection"})
    public double getDirection() {
        try {
            startBlockExecution(BlockType.GETTER, ".Direction");
            return this.compassSensor.getDirection();
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
    @Block(classes = {CompassSensor.class}, methodName = {"calibrationFailed"})
    public boolean getCalibrationFailed() {
        try {
            startBlockExecution(BlockType.GETTER, ".CalibrationFailed");
            return this.compassSensor.calibrationFailed();
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
    @Block(classes = {CompassSensor.class}, methodName = {"setMode"})
    public void setMode(String compassModeString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".Mode");
            CompassSensor.CompassMode compassMode = (CompassSensor.CompassMode) checkArg(compassModeString, CompassSensor.CompassMode.class, "compassMode");
            if (compassMode != null) {
                this.compassSensor.setMode(compassMode);
            }
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
