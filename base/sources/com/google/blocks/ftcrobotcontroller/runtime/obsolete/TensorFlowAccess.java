package com.google.blocks.ftcrobotcontroller.runtime.obsolete;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.runtime.Access;
import com.google.blocks.ftcrobotcontroller.runtime.BlockType;
import com.google.blocks.ftcrobotcontroller.runtime.BlocksOpMode;

/* JADX INFO: loaded from: classes8.dex */
public final class TensorFlowAccess extends Access {
    public TensorFlowAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "");
    }

    @JavascriptInterface
    public Object createBuilder() {
        handleObsoleteBlockExecution(BlockType.CREATE, "TfodProcessor.Builder", "");
        return null;
    }

    @JavascriptInterface
    public void setModelAssetName(Object tfodProcessorBuilderArg, String modelAssetName) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, "TfodProcessor.Builder", ".setModelAssetName");
    }

    @JavascriptInterface
    public void setModelFileName(Object tfodProcessorBuilderArg, String modelFileName) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, "TfodProcessor.Builder", ".setModelFileName");
    }

    @JavascriptInterface
    public void setModelLabels(Object tfodProcessorBuilderArg, String jsonLabels) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, "TfodProcessor.Builder", ".setModelLabels");
    }

    @JavascriptInterface
    public void setIsModelTensorFlow2(Object tfodProcessorBuilderArg, boolean isModelTensorFlow2) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, "TfodProcessor.Builder", ".setIsModelTensorFlow2");
    }

    @JavascriptInterface
    public void setIsModelQuantized(Object tfodProcessorBuilderArg, boolean isModelQuantized) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, "TfodProcessor.Builder", ".setIsModelQuantized");
    }

    @JavascriptInterface
    public void setModelInputSize(Object tfodProcessorBuilderArg, int modelInputSize) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, "TfodProcessor.Builder", ".setModelInputSize");
    }

    @JavascriptInterface
    public void setModelAspectRatio(Object tfodProcessorBuilderArg, double modelAspectRatio) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, "TfodProcessor.Builder", ".setModelAspectRatio");
    }

    @JavascriptInterface
    public void setNumDetectorThreads(Object tfodProcessorBuilderArg, int numDetectorThreads) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, "TfodProcessor.Builder", ".setNumDetectorThreads");
    }

    @JavascriptInterface
    public void setNumExecutorThreads(Object tfodProcessorBuilderArg, int numExecutorThreads) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, "TfodProcessor.Builder", ".setNumExecutorThreads");
    }

    @JavascriptInterface
    public void setMaxNumRecognitions(Object tfodProcessorBuilderArg, int maxNumRecognitions) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, "TfodProcessor.Builder", ".setMaxNumRecognitions");
    }

    @JavascriptInterface
    public void setUseObjectTracker(Object tfodProcessorBuilderArg, boolean useObjectTracker) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, "TfodProcessor.Builder", ".setUseObjectTracker");
    }

    @JavascriptInterface
    public void setTrackerMaxOverlap(Object tfodProcessorBuilderArg, float trackerMaxOverlap) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, "TfodProcessor.Builder", ".setTrackerMaxOverlap");
    }

    @JavascriptInterface
    public void setTrackerMinSize(Object tfodProcessorBuilderArg, float trackerMinSize) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, "TfodProcessor.Builder", ".setTrackerMinSize");
    }

    @JavascriptInterface
    public void setTrackerMarginalCorrelation(Object tfodProcessorBuilderArg, float trackerMarginalCorrelation) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, "TfodProcessor.Builder", ".setTrackerMarginalCorrelation");
    }

    @JavascriptInterface
    public void setTrackerMinCorrelation(Object tfodProcessorBuilderArg, float trackerMinCorrelation) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, "TfodProcessor.Builder", ".setTrackerMinCorrelation");
    }

    @JavascriptInterface
    public Object build(Object tfodProcessorBuilderArg) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, "TfodProcessor.Builder", ".build");
        return null;
    }

    @JavascriptInterface
    public Object easyCreateWithDefaults() {
        handleObsoleteBlockExecution(BlockType.FUNCTION, "TfodProcessor", ".easyCreateWithDefaults");
        return null;
    }

    @JavascriptInterface
    public void setMinResultConfidence(Object tfodProcessorArg, float minResultConfidence) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, "TfodProcessor", ".setMinResultConfidence");
    }

    @JavascriptInterface
    public void setClippingMargins(Object tfodProcessorArg, int left, int top, int right, int bottom) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, "TfodProcessor", ".setClippingMargins");
    }

    @JavascriptInterface
    public void setZoom(Object tfodProcessorArg, double magnification) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, "TfodProcessor", ".setZoom");
    }

    @JavascriptInterface
    public String getRecognitions(Object tfodProcessorArg) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, "TfodProcessor", ".getRecognitions");
        return "[]";
    }

    @JavascriptInterface
    public String getFreshRecognitions(Object tfodProcessorArg) {
        handleObsoleteBlockExecution(BlockType.FUNCTION, "TfodProcessor", ".getFreshRecognitions");
        return "[]";
    }
}
