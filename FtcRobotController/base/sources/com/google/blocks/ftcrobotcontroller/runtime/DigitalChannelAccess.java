package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

/* JADX INFO: loaded from: classes8.dex */
class DigitalChannelAccess extends HardwareAccess<DigitalChannel> {
    private final DigitalChannel digitalChannel;

    DigitalChannelAccess(BlocksOpMode blocksOpMode, HardwareItem hardwareItem, HardwareMap hardwareMap) {
        super(blocksOpMode, hardwareItem, hardwareMap, DigitalChannel.class);
        this.digitalChannel = (DigitalChannel) this.hardwareDevice;
    }

    @JavascriptInterface
    @Block(classes = {DigitalChannel.class}, methodName = {"setMode"})
    public void setMode(String modeString) {
        try {
            startBlockExecution(BlockType.SETTER, ".Mode");
            DigitalChannel.Mode mode = (DigitalChannel.Mode) checkArg(modeString, DigitalChannel.Mode.class, "");
            if (mode != null) {
                this.digitalChannel.setMode(mode);
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

    @JavascriptInterface
    @Block(classes = {DigitalChannel.class}, methodName = {"getMode"})
    public String getMode() {
        try {
            startBlockExecution(BlockType.GETTER, ".Mode");
            DigitalChannel.Mode mode = this.digitalChannel.getMode();
            if (mode != null) {
                return mode.toString();
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
    @Block(classes = {DigitalChannel.class}, methodName = {"setState"})
    public void setState(boolean state) {
        try {
            startBlockExecution(BlockType.SETTER, ".State");
            this.digitalChannel.setState(state);
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
    @Block(classes = {DigitalChannel.class}, methodName = {"getState"})
    public boolean getState() {
        try {
            startBlockExecution(BlockType.GETTER, ".State");
            return this.digitalChannel.getState();
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
