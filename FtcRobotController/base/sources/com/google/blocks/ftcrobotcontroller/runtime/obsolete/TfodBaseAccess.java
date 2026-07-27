package com.google.blocks.ftcrobotcontroller.runtime.obsolete;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.runtime.Access;
import com.google.blocks.ftcrobotcontroller.runtime.BlockType;
import com.google.blocks.ftcrobotcontroller.runtime.BlocksOpMode;

/* JADX INFO: loaded from: classes8.dex */
abstract class TfodBaseAccess extends Access {
    TfodBaseAccess(BlocksOpMode blocksOpMode, String identifier, String blockFirstName) {
        super(blocksOpMode, identifier, blockFirstName);
    }

    @JavascriptInterface
    public void initialize(Object vuforiaBaseAccess, float minimumConfidence, boolean useObjectTracker, boolean enableCameraMonitoring) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".initialize");
    }

    @JavascriptInterface
    public void initializeWithIsModelTensorFlow2(Object vuforiaBaseAccess, float minimumConfidence, boolean useObjectTracker, boolean enableCameraMonitoring, boolean isModelTensorFlow2) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".initialize");
    }

    @JavascriptInterface
    public void initializeWithAllArgs(Object vuforiaBaseAccess, float minimumConfidence, boolean useObjectTracker, boolean enableCameraMonitoring, boolean isModelTensorFlow2, boolean isModelQuantized, int inputSize, int numInterpreterThreads, int numExecutorThreads, int maxNumDetections, int timingBufferSize, double maxFrameRate, float trackerMaxOverlap, float trackerMinSize, float trackerMarginalCorrelation, float trackerMinCorrelation) {
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
    public void setClippingMargins(int left, int top, int right, int bottom) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".setClippingMargins");
    }

    @JavascriptInterface
    public void setZoom(double magnification, double aspectRatio) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".setZoom");
    }

    @JavascriptInterface
    public String getRecognitions() {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".getRecognitions");
        return "[]";
    }
}
