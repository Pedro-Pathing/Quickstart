package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/* JADX INFO: loaded from: classes8.dex */
class UltrasonicSensorAccess extends HardwareAccess<UltrasonicSensor> {
    private final UltrasonicSensor ultrasonicSensor;

    UltrasonicSensorAccess(BlocksOpMode blocksOpMode, HardwareItem hardwareItem, HardwareMap hardwareMap) {
        super(blocksOpMode, hardwareItem, hardwareMap, UltrasonicSensor.class);
        this.ultrasonicSensor = (UltrasonicSensor) this.hardwareDevice;
    }

    @JavascriptInterface
    @Block(classes = {UltrasonicSensor.class}, methodName = {"getUltrasonicLevel"})
    public double getUltrasonicLevel() {
        try {
            startBlockExecution(BlockType.GETTER, ".UltrasonicLevel");
            return this.ultrasonicSensor.getUltrasonicLevel();
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
