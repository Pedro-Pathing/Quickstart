package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

/* JADX INFO: loaded from: classes8.dex */
class BlinkinPatternAccess extends Access {
    BlinkinPatternAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "BlinkinPattern");
    }

    @JavascriptInterface
    @Block(classes = {RevBlinkinLedDriver.BlinkinPattern.class}, methodName = {"fromNumber"})
    public String fromNumber(int number) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".fromNumber");
            return RevBlinkinLedDriver.BlinkinPattern.fromNumber(number).toString();
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
    @Block(classes = {RevBlinkinLedDriver.BlinkinPattern.class}, methodName = {"ordinal"})
    public int toNumber(String blinkinPatternString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".toNumber");
            RevBlinkinLedDriver.BlinkinPattern blinkinPattern = checkBlinkinPattern(blinkinPatternString);
            if (blinkinPattern != null) {
                return blinkinPattern.ordinal();
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
    @Block(exclusiveToBlocks = true)
    public String fromText(String text) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".fromText");
            RevBlinkinLedDriver.BlinkinPattern blinkinPattern = checkBlinkinPattern(text);
            if (blinkinPattern != null) {
                return blinkinPattern.toString();
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
    @Block(classes = {RevBlinkinLedDriver.BlinkinPattern.class}, methodName = {"toString"})
    public String toText(String blinkinPatternString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".toText");
            RevBlinkinLedDriver.BlinkinPattern blinkinPattern = checkBlinkinPattern(blinkinPatternString);
            if (blinkinPattern != null) {
                return blinkinPattern.toString();
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
    @Block(classes = {RevBlinkinLedDriver.BlinkinPattern.class}, methodName = {"next"})
    public String next(String blinkinPatternString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".next");
            RevBlinkinLedDriver.BlinkinPattern blinkinPattern = checkBlinkinPattern(blinkinPatternString);
            if (blinkinPattern != null) {
                return blinkinPattern.next().toString();
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
    @Block(classes = {RevBlinkinLedDriver.BlinkinPattern.class}, methodName = {"previous"})
    public String previous(String blinkinPatternString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".previous");
            RevBlinkinLedDriver.BlinkinPattern blinkinPattern = checkBlinkinPattern(blinkinPatternString);
            if (blinkinPattern != null) {
                return blinkinPattern.previous().toString();
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
}
