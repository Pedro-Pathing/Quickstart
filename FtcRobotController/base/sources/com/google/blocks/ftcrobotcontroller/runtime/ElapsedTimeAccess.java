package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

/* JADX INFO: loaded from: classes8.dex */
class ElapsedTimeAccess extends Access {
    ElapsedTimeAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "ElapsedTime");
    }

    private ElapsedTime checkElapsedTime(Object elapsedTimeArg) {
        return (ElapsedTime) checkArg(elapsedTimeArg, ElapsedTime.class, "timer");
    }

    @JavascriptInterface
    @Block(classes = {ElapsedTime.class}, constructor = true)
    public ElapsedTime create() {
        try {
            startBlockExecution(BlockType.CREATE, "");
            return new ElapsedTime();
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
    @Block(classes = {ElapsedTime.class}, constructor = true)
    public ElapsedTime create_withStartTime(long startTime) {
        try {
            startBlockExecution(BlockType.CREATE, "");
            return new ElapsedTime(startTime);
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
    @Block(classes = {ElapsedTime.class}, constructor = true)
    public ElapsedTime create_withResolution(String resolutionString) {
        try {
            startBlockExecution(BlockType.CREATE, "");
            ElapsedTime.Resolution resolution = (ElapsedTime.Resolution) checkArg(resolutionString, ElapsedTime.Resolution.class, "resolution");
            if (resolution != null) {
                return new ElapsedTime(resolution);
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
    @Block(classes = {ElapsedTime.class}, methodName = {"startTime"})
    public double getStartTime(Object elapsedTimeArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".StartTime");
            ElapsedTime elapsedTime = checkElapsedTime(elapsedTimeArg);
            if (elapsedTime != null) {
                return elapsedTime.startTime();
            }
            endBlockExecution();
            return LynxServoController.apiPositionFirst;
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
    @Block(classes = {ElapsedTime.class}, methodName = {"time"})
    public double getTime(Object elapsedTimeArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".Time");
            ElapsedTime elapsedTime = checkElapsedTime(elapsedTimeArg);
            if (elapsedTime != null) {
                return elapsedTime.time();
            }
            endBlockExecution();
            return LynxServoController.apiPositionFirst;
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
    @Block(classes = {ElapsedTime.class}, methodName = {"seconds"})
    public double getSeconds(Object elapsedTimeArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".Seconds");
            ElapsedTime elapsedTime = checkElapsedTime(elapsedTimeArg);
            if (elapsedTime != null) {
                return elapsedTime.seconds();
            }
            endBlockExecution();
            return LynxServoController.apiPositionFirst;
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
    @Block(classes = {ElapsedTime.class}, methodName = {"milliseconds"})
    public double getMilliseconds(Object elapsedTimeArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".Milliseconds");
            ElapsedTime elapsedTime = checkElapsedTime(elapsedTimeArg);
            if (elapsedTime != null) {
                return elapsedTime.milliseconds();
            }
            endBlockExecution();
            return LynxServoController.apiPositionFirst;
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
    @Block(classes = {ElapsedTime.class}, methodName = {"getResolution"})
    public String getResolution(Object elapsedTimeArg) {
        ElapsedTime.Resolution resolution;
        try {
            startBlockExecution(BlockType.GETTER, ".Resolution");
            ElapsedTime elapsedTime = checkElapsedTime(elapsedTimeArg);
            if (elapsedTime != null && (resolution = elapsedTime.getResolution()) != null) {
                return resolution.toString();
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
    @Block(classes = {ElapsedTime.class}, methodName = {"toString"})
    public String getAsText(Object elapsedTimeArg) {
        try {
            startBlockExecution(BlockType.GETTER, ".AsText");
            ElapsedTime elapsedTime = checkElapsedTime(elapsedTimeArg);
            if (elapsedTime != null) {
                return elapsedTime.toString();
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
    @Block(classes = {ElapsedTime.class}, methodName = {"reset"})
    public void reset(Object elapsedTimeArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".reset");
            ElapsedTime elapsedTime = checkElapsedTime(elapsedTimeArg);
            if (elapsedTime != null) {
                elapsedTime.reset();
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
    @Block(classes = {ElapsedTime.class}, methodName = {"log"})
    public void log(Object elapsedTimeArg, String label) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".log");
            ElapsedTime elapsedTime = checkElapsedTime(elapsedTimeArg);
            if (elapsedTime != null) {
                elapsedTime.log(label);
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
    @Block(classes = {ElapsedTime.class}, methodName = {"toString"})
    public String toText(Object elapsedTimeArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".toText");
            ElapsedTime elapsedTime = checkElapsedTime(elapsedTimeArg);
            if (elapsedTime != null) {
                return elapsedTime.toString();
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
