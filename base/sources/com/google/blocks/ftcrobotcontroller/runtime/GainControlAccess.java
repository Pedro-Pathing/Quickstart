package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;

/* JADX INFO: loaded from: classes8.dex */
class GainControlAccess extends Access {
    GainControlAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "GainControl");
    }

    private GainControl checkGainControl(Object gainControlArg) {
        return (GainControl) checkArg(gainControlArg, GainControl.class, "gainControl");
    }

    @JavascriptInterface
    @Block(classes = {GainControl.class}, methodName = {"getMinGain"})
    public int getMinGain(Object gainControlArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "GainControl", ".getMinGain");
            GainControl gainControl = checkGainControl(gainControlArg);
            if (gainControl != null) {
                return gainControl.getMinGain();
            }
            endBlockExecution();
            return 0;
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
    @Block(classes = {GainControl.class}, methodName = {"getMaxGain"})
    public int getMaxGain(Object gainControlArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "GainControl", ".getMaxGain");
            GainControl gainControl = checkGainControl(gainControlArg);
            if (gainControl != null) {
                return gainControl.getMaxGain();
            }
            endBlockExecution();
            return 0;
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
    @Block(classes = {GainControl.class}, methodName = {"getGain"})
    public int getGain(Object gainControlArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "GainControl", ".getGain");
            GainControl gainControl = checkGainControl(gainControlArg);
            if (gainControl != null) {
                return gainControl.getGain();
            }
            endBlockExecution();
            return 0;
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
    @Block(classes = {GainControl.class}, methodName = {"setGain"})
    public boolean setGain(Object gainControlArg, int gain) {
        try {
            startBlockExecution(BlockType.FUNCTION, "GainControl", ".setGain");
            GainControl gainControl = checkGainControl(gainControlArg);
            if (gainControl != null) {
                return gainControl.setGain(gain);
            }
            endBlockExecution();
            return false;
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
