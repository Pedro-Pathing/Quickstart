package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

/* JADX INFO: loaded from: classes8.dex */
class TouchSensorAccess extends HardwareAccess<TouchSensor> {
    private final TouchSensor touchSensor;

    TouchSensorAccess(BlocksOpMode blocksOpMode, HardwareItem hardwareItem, HardwareMap hardwareMap) {
        super(blocksOpMode, hardwareItem, hardwareMap, TouchSensor.class);
        this.touchSensor = (TouchSensor) this.hardwareDevice;
    }

    @JavascriptInterface
    @Block(classes = {ModernRoboticsTouchSensor.class, RevTouchSensor.class, TouchSensor.class}, methodName = {"isPressed"})
    public boolean getIsPressed() {
        try {
            startBlockExecution(BlockType.GETTER, ".IsPressed");
            return this.touchSensor.isPressed();
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
