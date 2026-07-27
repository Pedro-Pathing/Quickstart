package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;

/* JADX INFO: loaded from: classes8.dex */
class PtzControlAccess extends Access {
    PtzControlAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "PtzControl");
    }

    private PtzControl checkPtzControl(Object ptzControlArg) {
        return (PtzControl) checkArg(ptzControlArg, PtzControl.class, "ptzControl");
    }

    @JavascriptInterface
    @Block(classes = {PtzControl.class}, methodName = {"getMinPanTilt"})
    public String getMinPanTilt(Object ptzControlArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "PtzControl", ".getMinPanTilt");
            PtzControl ptzControl = checkPtzControl(ptzControlArg);
            if (ptzControl != null) {
                return SimpleGson.getInstance().toJson(ptzControl.getMinPanTilt());
            }
            return "{\"pan\": 0, \"tilt\": 0}";
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
    @Block(classes = {PtzControl.class}, methodName = {"getMaxPanTilt"})
    public String getMaxPanTilt(Object ptzControlArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "PtzControl", ".getMaxPanTilt");
            PtzControl ptzControl = checkPtzControl(ptzControlArg);
            if (ptzControl != null) {
                return SimpleGson.getInstance().toJson(ptzControl.getMaxPanTilt());
            }
            return "{\"pan\": 0, \"tilt\": 0}";
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
    @Block(classes = {PtzControl.class}, methodName = {"getPanTilt"})
    public String getPanTilt(Object ptzControlArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "PtzControl", ".getPanTilt");
            PtzControl ptzControl = checkPtzControl(ptzControlArg);
            if (ptzControl != null) {
                return SimpleGson.getInstance().toJson(ptzControl.getPanTilt());
            }
            return "{\"pan\": 0, \"tilt\": 0}";
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
    @Block(classes = {PtzControl.class}, methodName = {"setPanTilt"})
    public boolean setPanTilt(Object ptzControlArg, String json) {
        try {
            startBlockExecution(BlockType.FUNCTION, "PtzControl", ".setPanTilt");
            PtzControl ptzControl = checkPtzControl(ptzControlArg);
            if (ptzControl != null) {
                PtzControl.PanTiltHolder panTiltHolder = (PtzControl.PanTiltHolder) SimpleGson.getInstance().fromJson(json, PtzControl.PanTiltHolder.class);
                return ptzControl.setPanTilt(panTiltHolder);
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
    @Block(classes = {PtzControl.class}, methodName = {"getMinZoom"})
    public int getMinZoom(Object ptzControlArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "PtzControl", ".getMinZoom");
            PtzControl ptzControl = checkPtzControl(ptzControlArg);
            if (ptzControl != null) {
                return ptzControl.getMinZoom();
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
    @Block(classes = {PtzControl.class}, methodName = {"getMaxZoom"})
    public int getMaxZoom(Object ptzControlArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "PtzControl", ".getMaxZoom");
            PtzControl ptzControl = checkPtzControl(ptzControlArg);
            if (ptzControl != null) {
                return ptzControl.getMaxZoom();
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
    @Block(classes = {PtzControl.class}, methodName = {"getZoom"})
    public int getZoom(Object ptzControlArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "PtzControl", ".getZoom");
            PtzControl ptzControl = checkPtzControl(ptzControlArg);
            if (ptzControl != null) {
                return ptzControl.getZoom();
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
    @Block(classes = {PtzControl.class}, methodName = {"setZoom"})
    public boolean setZoom(Object ptzControlArg, int zoom) {
        try {
            startBlockExecution(BlockType.FUNCTION, "PtzControl", ".setZoom");
            PtzControl ptzControl = checkPtzControl(ptzControlArg);
            if (ptzControl != null) {
                return ptzControl.setZoom(zoom);
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
