package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

/* JADX INFO: loaded from: classes8.dex */
class RevBlinkinLedDriverAccess extends HardwareAccess<RevBlinkinLedDriver> {
    private final RevBlinkinLedDriver revBlinkinLedDriver;

    RevBlinkinLedDriverAccess(BlocksOpMode blocksOpMode, HardwareItem hardwareItem, HardwareMap hardwareMap) {
        super(blocksOpMode, hardwareItem, hardwareMap, RevBlinkinLedDriver.class);
        this.revBlinkinLedDriver = (RevBlinkinLedDriver) this.hardwareDevice;
    }

    @JavascriptInterface
    @Block(classes = {RevBlinkinLedDriver.class}, methodName = {"setPattern"})
    public void setPattern(String blinkinPatternString) {
        try {
            startBlockExecution(BlockType.SETTER, ".Pattern");
            RevBlinkinLedDriver.BlinkinPattern blinkinPattern = checkBlinkinPattern(blinkinPatternString);
            if (blinkinPattern != null) {
                this.revBlinkinLedDriver.setPattern(blinkinPattern);
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
