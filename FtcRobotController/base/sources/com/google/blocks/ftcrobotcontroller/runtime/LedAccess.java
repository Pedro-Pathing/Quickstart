package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

/* JADX INFO: loaded from: classes8.dex */
class LedAccess extends HardwareAccess<LED> {
    private final LED led;

    LedAccess(BlocksOpMode blocksOpMode, HardwareItem hardwareItem, HardwareMap hardwareMap) {
        super(blocksOpMode, hardwareItem, hardwareMap, LED.class);
        this.led = (LED) this.hardwareDevice;
    }

    @JavascriptInterface
    @Block(classes = {LED.class}, methodName = {"enable"})
    public void enableLed(boolean enable) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".enableLed");
            this.led.enable(enable);
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
    @Block(classes = {LED.class}, methodName = {"isLightOn"})
    public boolean isLightOn() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".isLightOn");
            return this.led.isLightOn();
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
    @Block(classes = {LED.class}, methodName = {"on"})
    public void on() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".on");
            this.led.on();
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
    @Block(classes = {LED.class}, methodName = {"off"})
    public void off() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".off");
            this.led.off();
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
