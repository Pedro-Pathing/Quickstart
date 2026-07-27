package com.google.blocks.ftcrobotcontroller.runtime.obsolete;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.runtime.Access;
import com.google.blocks.ftcrobotcontroller.runtime.BlockType;
import com.google.blocks.ftcrobotcontroller.runtime.BlocksOpMode;

/* JADX INFO: loaded from: classes8.dex */
public final class TfodAccess extends Access {
    public TfodAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "TensorFlowObjectDetection");
    }

    @JavascriptInterface
    public void useDefaultModel() {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".useDefaultModelForPOWERPLAY");
    }

    @JavascriptInterface
    public void useModelFromAsset(String assetName, String jsonLabels, boolean isModelTensorFlow2, boolean isModelQuantized, int inputSize) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".useModelFromAsset");
    }

    @JavascriptInterface
    public void useModelFromFile(String fileName, String jsonLabels, boolean isModelTensorFlow2, boolean isModelQuantized, int inputSize) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".useModelFromFile");
    }

    @JavascriptInterface
    public void initialize(Object vuforiaBaseAccess, float minimumConfidence, boolean useObjectTracker, boolean enableCameraMonitoring) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".initialize");
    }

    @JavascriptInterface
    public void initializeWithMoreArgs(Object vuforiaBaseAccess, float minimumConfidence, boolean useObjectTracker, boolean enableCameraMonitoring, int numInterpreterThreads, int numExecutorThreads, int maxNumDetections, int timingBufferSize, double maxFrameRate, float trackerMaxOverlap, float trackerMinSize, float trackerMarginalCorrelation, float trackerMinCorrelation) {
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

    @JavascriptInterface
    public void setModelFromAssetLegacy(String assetName, String jsonLabels) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".useModelFromAsset");
    }

    @JavascriptInterface
    public void setModelFromFileLegacy(String fileName, String jsonLabels) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".useModelFromFile");
    }

    @JavascriptInterface
    public void initializeWithIsModelTensorFlow2Legacy(Object vuforiaBaseAccess, float minimumConfidence, boolean useObjectTracker, boolean enableCameraMonitoring, boolean isModelTensorFlow2) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".initialize");
    }

    @JavascriptInterface
    public void initializeWithAllArgsLegacy(Object vuforiaBaseAccess, float minimumConfidence, boolean useObjectTracker, boolean enableCameraMonitoring, boolean isModelTensorFlow2, boolean isModelQuantized, int inputSize, int numInterpreterThreads, int numExecutorThreads, int maxNumDetections, int timingBufferSize, double maxFrameRate, float trackerMaxOverlap, float trackerMinSize, float trackerMarginalCorrelation, float trackerMinCorrelation) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, ".initialize");
    }
}
