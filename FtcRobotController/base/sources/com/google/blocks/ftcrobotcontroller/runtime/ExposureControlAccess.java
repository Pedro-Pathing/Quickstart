package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;

/* JADX INFO: loaded from: classes8.dex */
class ExposureControlAccess extends Access {
    ExposureControlAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "ExposureControl");
    }

    private ExposureControl checkExposureControl(Object exposureControlArg) {
        return (ExposureControl) checkArg(exposureControlArg, ExposureControl.class, "exposureControl");
    }

    private ExposureControl.Mode checkMode(String modeString) {
        return (ExposureControl.Mode) checkArg(modeString, ExposureControl.Mode.class, "Mode");
    }

    private TimeUnit checkTimeUnit(String timeUnitString) {
        return (TimeUnit) checkArg(timeUnitString, TimeUnit.class, "timeUnit");
    }

    @JavascriptInterface
    @Block(classes = {ExposureControl.class}, methodName = {"getMode"})
    public String getMode(Object exposureControlArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getMode");
            ExposureControl exposureControl = checkExposureControl(exposureControlArg);
            if (exposureControl != null) {
                return exposureControl.getMode().toString();
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
    @Block(classes = {ExposureControl.class}, methodName = {"setMode"})
    public boolean setMode(Object exposureControlArg, String modeString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setMode");
            ExposureControl exposureControl = checkExposureControl(exposureControlArg);
            ExposureControl.Mode mode = checkMode(modeString);
            if (exposureControl != null && mode != null) {
                return exposureControl.setMode(mode);
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
    @Block(classes = {ExposureControl.class}, methodName = {"isModeSupported"})
    public boolean isModeSupported(Object exposureControlArg, String modeString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".isModeSupported");
            ExposureControl exposureControl = checkExposureControl(exposureControlArg);
            ExposureControl.Mode mode = checkMode(modeString);
            if (exposureControl != null && mode != null) {
                return exposureControl.isModeSupported(mode);
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
    @Block(classes = {ExposureControl.class}, methodName = {"getMinExposure"})
    public long getMinExposure(Object exposureControlArg, String timeUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getMinExposure");
            ExposureControl exposureControl = checkExposureControl(exposureControlArg);
            TimeUnit timeUnit = checkTimeUnit(timeUnitString);
            if (exposureControl != null && timeUnit != null) {
                return exposureControl.getMinExposure(timeUnit);
            }
            endBlockExecution();
            return 0L;
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
    @Block(classes = {ExposureControl.class}, methodName = {"getMaxExposure"})
    public long getMaxExposure(Object exposureControlArg, String timeUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getMaxExposure");
            ExposureControl exposureControl = checkExposureControl(exposureControlArg);
            TimeUnit timeUnit = checkTimeUnit(timeUnitString);
            if (exposureControl != null && timeUnit != null) {
                return exposureControl.getMaxExposure(timeUnit);
            }
            endBlockExecution();
            return 0L;
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
    @Block(classes = {ExposureControl.class}, methodName = {"getExposure"})
    public long getExposure(Object exposureControlArg, String timeUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getExposure");
            ExposureControl exposureControl = checkExposureControl(exposureControlArg);
            TimeUnit timeUnit = checkTimeUnit(timeUnitString);
            if (exposureControl != null && timeUnit != null) {
                return exposureControl.getExposure(timeUnit);
            }
            endBlockExecution();
            return 0L;
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
    @Block(classes = {ExposureControl.class}, methodName = {"setExposure"})
    public boolean setExposure(Object exposureControlArg, long duration, String timeUnitString) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setExposure");
            ExposureControl exposureControl = checkExposureControl(exposureControlArg);
            TimeUnit timeUnit = checkTimeUnit(timeUnitString);
            if (exposureControl != null && timeUnit != null) {
                return exposureControl.setExposure(duration, timeUnit);
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
    @Block(classes = {ExposureControl.class}, methodName = {"isExposureSupported"})
    public boolean isExposureSupported(Object exposureControlArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".isExposureSupported");
            ExposureControl exposureControl = checkExposureControl(exposureControlArg);
            if (exposureControl != null) {
                return exposureControl.isExposureSupported();
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
    @Block(classes = {ExposureControl.class}, methodName = {"getAePriority"})
    public boolean getAePriority(Object exposureControlArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getAePriority");
            ExposureControl exposureControl = checkExposureControl(exposureControlArg);
            if (exposureControl != null) {
                return exposureControl.getAePriority();
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
    @Block(classes = {ExposureControl.class}, methodName = {"setAePriority"})
    public boolean setAePriority(Object exposureControlArg, boolean priority) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setAePriority");
            ExposureControl exposureControl = checkExposureControl(exposureControlArg);
            if (exposureControl != null) {
                return exposureControl.setAePriority(priority);
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
