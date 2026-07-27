package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;

/* JADX INFO: loaded from: classes8.dex */
class WhiteBalanceControlAccess extends Access {
    WhiteBalanceControlAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "WhiteBalanceControl");
    }

    private WhiteBalanceControl checkWhiteBalanceControl(Object whiteBalanceControlArg) {
        return (WhiteBalanceControl) checkArg(whiteBalanceControlArg, WhiteBalanceControl.class, "whiteBalanceControl");
    }

    private WhiteBalanceControl.Mode checkMode(String modeString) {
        return (WhiteBalanceControl.Mode) checkArg(modeString, WhiteBalanceControl.Mode.class, "Mode");
    }

    @JavascriptInterface
    @Block(classes = {WhiteBalanceControl.class}, methodName = {"getMode"})
    public String getMode(Object whiteBalanceControlArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "WhiteBalanceControl", ".getMode");
            WhiteBalanceControl whiteBalanceControl = checkWhiteBalanceControl(whiteBalanceControlArg);
            if (whiteBalanceControl != null) {
                return whiteBalanceControl.getMode().toString();
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
    @Block(classes = {WhiteBalanceControl.class}, methodName = {"setMode"})
    public boolean setMode(Object whiteBalanceControlArg, String modeString) {
        try {
            startBlockExecution(BlockType.FUNCTION, "WhiteBalanceControl", ".setMode");
            WhiteBalanceControl whiteBalanceControl = checkWhiteBalanceControl(whiteBalanceControlArg);
            WhiteBalanceControl.Mode mode = checkMode(modeString);
            if (whiteBalanceControl != null && mode != null) {
                return whiteBalanceControl.setMode(mode);
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

    @JavascriptInterface
    @Block(classes = {WhiteBalanceControl.class}, methodName = {"getMinWhiteBalanceTemperature"})
    public int getMinWhiteBalanceTemperature(Object whiteBalanceControlArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "WhiteBalanceControl", ".getMinWhiteBalanceTemperature");
            WhiteBalanceControl whiteBalanceControl = checkWhiteBalanceControl(whiteBalanceControlArg);
            if (whiteBalanceControl != null) {
                return whiteBalanceControl.getMinWhiteBalanceTemperature();
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
    @Block(classes = {WhiteBalanceControl.class}, methodName = {"getMaxWhiteBalanceTemperature"})
    public int getMaxWhiteBalanceTemperature(Object whiteBalanceControlArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "WhiteBalanceControl", ".getMaxWhiteBalanceTemperature");
            WhiteBalanceControl whiteBalanceControl = checkWhiteBalanceControl(whiteBalanceControlArg);
            if (whiteBalanceControl != null) {
                return whiteBalanceControl.getMaxWhiteBalanceTemperature();
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
    @Block(classes = {WhiteBalanceControl.class}, methodName = {"getWhiteBalanceTemperature"})
    public int getWhiteBalanceTemperature(Object whiteBalanceControlArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "WhiteBalanceControl", ".getWhiteBalanceTemperature");
            WhiteBalanceControl whiteBalanceControl = checkWhiteBalanceControl(whiteBalanceControlArg);
            if (whiteBalanceControl != null) {
                return whiteBalanceControl.getWhiteBalanceTemperature();
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
    @Block(classes = {WhiteBalanceControl.class}, methodName = {"setWhiteBalanceTemperature"})
    public boolean setWhiteBalanceTemperature(Object whiteBalanceControlArg, int whiteBalanceTemperature) {
        try {
            startBlockExecution(BlockType.FUNCTION, "WhiteBalanceControl", ".setWhiteBalanceTemperature");
            WhiteBalanceControl whiteBalanceControl = checkWhiteBalanceControl(whiteBalanceControlArg);
            if (whiteBalanceControl != null) {
                return whiteBalanceControl.setWhiteBalanceTemperature(whiteBalanceTemperature);
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
