package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.qualcomm.hardware.limelightvision.LLStatus;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

/* JADX INFO: loaded from: classes8.dex */
class LLStatusAccess extends Access {
    LLStatusAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "LLStatus");
    }

    private LLStatus checkLLStatus(Object llStatusArg) {
        return (LLStatus) checkArg(llStatusArg, LLStatus.class, "llStatus");
    }

    @JavascriptInterface
    @Block(classes = {LLStatus.class}, methodName = {"getCameraQuat"})
    public Quaternion getCameraQuat(Object llStatusArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getCameraQuat");
            LLStatus llStatus = checkLLStatus(llStatusArg);
            if (llStatusArg != null) {
                return llStatus.getCameraQuat();
            }
            endBlockExecution();
            return null;
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
    @Block(classes = {LLStatus.class}, methodName = {"toString"})
    public String toText(Object llStatusArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".toText");
            LLStatus llStatus = checkLLStatus(llStatusArg);
            if (llStatusArg != null) {
                return llStatus.toString();
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
