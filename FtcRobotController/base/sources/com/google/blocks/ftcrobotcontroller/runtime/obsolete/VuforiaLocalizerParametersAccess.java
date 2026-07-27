package com.google.blocks.ftcrobotcontroller.runtime.obsolete;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.runtime.Access;
import com.google.blocks.ftcrobotcontroller.runtime.BlockType;
import com.google.blocks.ftcrobotcontroller.runtime.BlocksOpMode;

/* JADX INFO: loaded from: classes8.dex */
public class VuforiaLocalizerParametersAccess extends Access {
    public VuforiaLocalizerParametersAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "VuforiaLocalizer.Parameters");
    }

    @JavascriptInterface
    public Object create() {
        handleObsoleteBlockExecution(BlockType.CREATE, "");
        return null;
    }

    @JavascriptInterface
    public void setVuforiaLicenseKey(Object parametersArg, String vuforiaLicenseKey) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".setVuforiaLicenseKey");
    }

    @JavascriptInterface
    public String getVuforiaLicenseKey(Object parametersArg) {
        handleObsoleteBlockExecution(BlockType.GETTER, ".VuforiaLicenseKey");
        return "";
    }

    @JavascriptInterface
    public void setCameraDirection(Object parametersArg, String cameraDirectionString) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".setCameraDirection");
    }

    @JavascriptInterface
    public String getCameraDirection(Object parametersArg) {
        handleObsoleteBlockExecution(BlockType.GETTER, ".CameraDirection");
        return "";
    }

    @JavascriptInterface
    public void setCameraName(Object parametersArg, String cameraNameString) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".setCameraName");
    }

    @JavascriptInterface
    public String getCameraName(Object parametersArg) {
        handleObsoleteBlockExecution(BlockType.GETTER, ".CameraName");
        return "";
    }

    @JavascriptInterface
    public void addWebcamCalibrationFile(Object parametersArg, String webcamCalibrationFilename) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".addWebcamCalibrationFile");
    }

    @JavascriptInterface
    public void setUseExtendedTracking(Object parametersArg, boolean useExtendedTracking) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".setUseExtendedTracking");
    }

    @JavascriptInterface
    public boolean getUseExtendedTracking(Object parametersArg) {
        handleObsoleteBlockExecution(BlockType.GETTER, ".UseExtendedTracking");
        return false;
    }

    @JavascriptInterface
    public boolean getEnableCameraMonitoring(Object parametersArg) {
        handleObsoleteBlockExecution(BlockType.GETTER, ".EnableCameraMonitoring");
        return false;
    }

    @JavascriptInterface
    public void setCameraMonitorFeedback(Object parametersArg, String cameraMonitorFeedbackString) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".setCameraMonitorFeedback");
    }

    @JavascriptInterface
    public String getCameraMonitorFeedback(Object parametersArg) {
        handleObsoleteBlockExecution(BlockType.GETTER, ".CameraMonitorFeedback");
        return "";
    }

    @JavascriptInterface
    public void setFillCameraMonitorViewParent(Object parametersArg, boolean fillCameraMonitorViewParent) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".setFillCameraMonitorViewParent");
    }

    @JavascriptInterface
    public boolean getFillCameraMonitorViewParent(Object parametersArg) {
        handleObsoleteBlockExecution(BlockType.GETTER, ".FillCameraMonitorViewParent");
        return false;
    }

    @JavascriptInterface
    public void setEnableCameraMonitoring(Object parametersArg, boolean enableCameraMonitoring) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".setEnableCameraMonitoring");
    }
}
