package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;

/* JADX INFO: loaded from: classes8.dex */
class Limelight3AAccess extends HardwareAccess<Limelight3A> {
    private final Limelight3A limelight3a;

    Limelight3AAccess(BlocksOpMode blocksOpMode, HardwareItem hardwareItem, HardwareMap hardwareMap) {
        super(blocksOpMode, hardwareItem, hardwareMap, Limelight3A.class);
        this.limelight3a = (Limelight3A) this.hardwareDevice;
    }

    @JavascriptInterface
    @Block(classes = {Limelight3A.class}, methodName = {"start"})
    public void start() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".start");
            this.limelight3a.start();
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
    @Block(classes = {Limelight3A.class}, methodName = {"pause"})
    public void pause() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".pause");
            this.limelight3a.pause();
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
    @Block(classes = {Limelight3A.class}, methodName = {"stop"})
    public void stop() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".stop");
            this.limelight3a.stop();
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
    @Block(classes = {Limelight3A.class}, methodName = {"isRunning"})
    public boolean isRunning() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".isRunning");
            return this.limelight3a.isRunning();
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
    @Block(classes = {Limelight3A.class}, methodName = {"setPollRateHz"})
    public void setPollRateHz(int rateHz) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setPollRateHz");
            this.limelight3a.setPollRateHz(rateHz);
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
    @Block(classes = {Limelight3A.class}, methodName = {"getTimeSinceLastUpdate"})
    public long getTimeSinceLastUpdate() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getTimeSinceLastUpdate");
            return this.limelight3a.getTimeSinceLastUpdate();
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
    @Block(classes = {Limelight3A.class}, methodName = {"isConnected"})
    public boolean isConnected() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".isConnected");
            return this.limelight3a.isConnected();
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
    @Block(classes = {Limelight3A.class}, methodName = {"getLatestResult"})
    public Object getLatestResult() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getLatestResult");
            return this.limelight3a.getLatestResult();
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
    @Block(classes = {Limelight3A.class}, methodName = {"getStatus"})
    public Object getStatus() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getStatus");
            return this.limelight3a.getStatus();
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
    @Block(classes = {Limelight3A.class}, methodName = {"reloadPipeline"})
    public boolean reloadPipeline() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".reloadPipeline");
            return this.limelight3a.reloadPipeline();
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
    @Block(classes = {Limelight3A.class}, methodName = {"pipelineSwitch"})
    public boolean pipelineSwitch(int index) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".pipelineSwitch");
            return this.limelight3a.pipelineSwitch(index);
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
    @Block(classes = {Limelight3A.class}, methodName = {"updatePythonInputs"})
    public boolean updatePythonInputs_with8Doubles(double input1, double input2, double input3, double input4, double input5, double input6, double input7, double input8) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".updatePythonInputs");
            return this.limelight3a.updatePythonInputs(input1, input2, input3, input4, input5, input6, input7, input8);
        } finally {
        }
    }

    @JavascriptInterface
    @Block(classes = {Limelight3A.class}, methodName = {"updatePythonInputs"})
    public boolean updatePythonInputs_withArray(String jsonInputs) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".updatePythonInputs");
            double[] inputs = (double[]) SimpleGson.getInstance().fromJson(jsonInputs, double[].class);
            return this.limelight3a.updatePythonInputs(inputs);
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
    @Block(classes = {Limelight3A.class}, methodName = {"updateRobotOrientation"})
    public boolean updateRobotOrientation(double yaw) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".updateRobotOrientation");
            return this.limelight3a.updateRobotOrientation(yaw);
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
    @Block(classes = {Limelight3A.class}, methodName = {"shutdown"})
    public void shutdown() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".shutdown");
            this.limelight3a.shutdown();
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
