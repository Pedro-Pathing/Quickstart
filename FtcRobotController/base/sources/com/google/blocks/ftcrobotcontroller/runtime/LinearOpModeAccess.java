package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/* JADX INFO: loaded from: classes8.dex */
class LinearOpModeAccess extends Access {
    private final BlocksOpMode blocksOpMode;

    LinearOpModeAccess(BlocksOpMode blocksOpMode, String identifier, String projectName) {
        super(blocksOpMode, identifier, projectName);
        this.blocksOpMode = blocksOpMode;
    }

    @JavascriptInterface
    @Block(classes = {LinearOpMode.class}, methodName = {"waitForStart"})
    public void waitForStart() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".waitForStart");
            this.blocksOpMode.waitForStartForBlocks();
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
    @Block(classes = {LinearOpMode.class}, methodName = {"idle"})
    public void idle() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".idle");
            this.blocksOpMode.idle();
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
    @Block(classes = {LinearOpMode.class}, methodName = {"sleep"})
    public void sleep(double millis) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".sleep");
            this.blocksOpMode.sleepForBlocks((long) millis);
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
    @Block(classes = {LinearOpMode.class}, methodName = {"opModeInInit"})
    public boolean opModeInInit() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".opModeInInit");
            return this.blocksOpMode.opModeInInit();
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
    @Block(classes = {LinearOpMode.class}, methodName = {"opModeIsActive"})
    public boolean opModeIsActive() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".opModeIsActive");
            return this.blocksOpMode.opModeIsActive();
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
    @Block(classes = {LinearOpMode.class}, methodName = {"isStarted"})
    public boolean isStarted() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".isStarted");
            return this.blocksOpMode.isStartedForBlocks();
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
    @Block(classes = {LinearOpMode.class}, methodName = {"isStopRequested"})
    public boolean isStopRequested() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".isStopRequested");
            return this.blocksOpMode.isStopRequestedForBlocks();
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
    @Block(classes = {LinearOpMode.class, OpMode.class}, methodName = {"getRuntime"})
    public double getRuntime() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getRuntime");
            return this.blocksOpMode.getRuntime();
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
    @Block(classes = {LinearOpMode.class, OpMode.class}, methodName = {"resetRuntime"})
    public void resetRuntime() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".resetRuntime");
            this.blocksOpMode.resetRuntime();
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
    @Block(classes = {LinearOpMode.class, OpMode.class}, methodName = {"requestOpModeStop"})
    public void requestOpModeStop() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".requestOpModeStop");
            this.blocksOpMode.requestOpModeStop();
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
    @Block(classes = {LinearOpMode.class, OpMode.class}, methodName = {"terminateOpModeNow"})
    public void terminateOpModeNow() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".terminateOpModeNow");
            this.blocksOpMode.terminateOpModeNowForBlocks();
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
