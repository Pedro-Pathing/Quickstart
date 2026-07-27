package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;

/* JADX INFO: loaded from: classes8.dex */
class ServoControllerAccess extends HardwareAccess<ServoController> {
    private final ServoController servoController;

    ServoControllerAccess(BlocksOpMode blocksOpMode, HardwareItem hardwareItem, HardwareMap hardwareMap) {
        super(blocksOpMode, hardwareItem, hardwareMap, ServoController.class);
        this.servoController = (ServoController) this.hardwareDevice;
    }

    @JavascriptInterface
    @Block(classes = {ServoController.class}, methodName = {"getPwmStatus"})
    public String getPwmStatus() {
        try {
            startBlockExecution(BlockType.GETTER, ".PwmStatus");
            ServoController.PwmStatus pwmStatus = this.servoController.getPwmStatus();
            if (pwmStatus != null) {
                return pwmStatus.toString();
            }
            return "";
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
    @Block(classes = {ServoController.class}, methodName = {"pwmEnable"})
    public void pwmEnable() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".pwmEnable");
            this.servoController.pwmEnable();
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
    @Block(classes = {ServoController.class}, methodName = {"pwmDisable"})
    public void pwmDisable() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".pwmDisable");
            this.servoController.pwmDisable();
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
