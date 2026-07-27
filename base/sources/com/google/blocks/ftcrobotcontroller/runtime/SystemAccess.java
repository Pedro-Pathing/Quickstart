package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;

/* JADX INFO: loaded from: classes8.dex */
class SystemAccess extends Access {
    SystemAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "System");
    }

    @JavascriptInterface
    public long nanoTime() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".nanoTime");
            return System.nanoTime();
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
    public long currentTimeMillis() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".currentTimeMillis");
            return System.currentTimeMillis();
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
