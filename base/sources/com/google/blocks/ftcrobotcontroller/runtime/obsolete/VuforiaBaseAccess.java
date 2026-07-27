package com.google.blocks.ftcrobotcontroller.runtime.obsolete;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.runtime.Access;
import com.google.blocks.ftcrobotcontroller.runtime.BlockType;
import com.google.blocks.ftcrobotcontroller.runtime.BlocksOpMode;

/* JADX INFO: loaded from: classes8.dex */
abstract class VuforiaBaseAccess extends Access {
    VuforiaBaseAccess(BlocksOpMode blocksOpMode, String identifier, String blockFirstName) {
        super(blocksOpMode, identifier, blockFirstName);
    }

    @JavascriptInterface
    public void initialize_withCameraDirection(String vuforiaLicenseKey, String cameraDirectionString, boolean useExtendedTracking, boolean enableCameraMonitoring, String cameraMonitorFeedbackString, float dx, float dy, float dz, float xAngle, float yAngle, float zAngle, boolean useCompetitionFieldTargetLocations) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".initialize");
    }

    @JavascriptInterface
    public void initialize_withCameraDirection_2(String cameraDirectionString, boolean useExtendedTracking, boolean enableCameraMonitoring, String cameraMonitorFeedbackString, float dx, float dy, float dz, String axesOrderString, float firstAngle, float secondAngle, float thirdAngle, boolean useCompetitionFieldTargetLocations) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".initialize");
    }

    @JavascriptInterface
    public void initialize_withWebcam(String cameraNameString, String webcamCalibrationFilename, boolean useExtendedTracking, boolean enableCameraMonitoring, String cameraMonitorFeedbackString, float dx, float dy, float dz, float xAngle, float yAngle, float zAngle, boolean useCompetitionFieldTargetLocations) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".initialize");
    }

    @JavascriptInterface
    public void initialize_withWebcam_2(String cameraNameString, String webcamCalibrationFilename, boolean useExtendedTracking, boolean enableCameraMonitoring, String cameraMonitorFeedbackString, float dx, float dy, float dz, String axesOrderString, float firstAngle, float secondAngle, float thirdAngle, boolean useCompetitionFieldTargetLocations) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".initialize");
    }

    @JavascriptInterface
    public void activate() {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".activate");
    }

    @JavascriptInterface
    public void deactivate() {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".deactivate");
    }

    @JavascriptInterface
    public void setActiveCamera(String cameraNameString) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".setActiveCamera");
    }

    @JavascriptInterface
    public String track(String name) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".track");
        return "[]";
    }

    @JavascriptInterface
    public String trackPose(String name) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".trackPose");
        return "[]";
    }

    @JavascriptInterface
    public Object getVuforiaLocalizer() {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".getVuforiaLocalizer");
        return null;
    }
}
