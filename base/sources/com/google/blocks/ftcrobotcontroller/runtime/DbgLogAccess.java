package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.qualcomm.robotcore.util.RobotLog;

/* JADX INFO: loaded from: classes8.dex */
class DbgLogAccess extends Access {
    public static final String TAG = "DbgLog";

    DbgLogAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, TAG);
    }

    @JavascriptInterface
    public void msg(String message) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".msg");
            RobotLog.ii(TAG, message);
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
    public void error(String message) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".error");
            RobotLog.ee(TAG, message);
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
