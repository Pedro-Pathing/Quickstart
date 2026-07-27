package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.qualcomm.robotcore.util.Range;

/* JADX INFO: loaded from: classes8.dex */
class RangeAccess extends Access {
    RangeAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "Range");
    }

    @JavascriptInterface
    @Block(classes = {Range.class}, methodName = {"clip"})
    public double clip(double number, double min, double max) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".clip");
            return Range.clip(number, min, max);
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
    @Block(classes = {Range.class}, methodName = {"scale"})
    public double scale(double number, double x1, double x2, double y1, double y2) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".scale");
            return Range.scale(number, x1, x2, y1, y2);
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
