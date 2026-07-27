package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

/* JADX INFO: loaded from: classes8.dex */
class PredominantColorAccess extends Access {
    PredominantColorAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "");
    }

    private PredominantColorProcessor.Builder checkPredominantColorProcessorBuilder(Object predominantColorProcessorBuilderArg) {
        return (PredominantColorProcessor.Builder) checkArg(predominantColorProcessorBuilderArg, PredominantColorProcessor.Builder.class, "predominantColorProcessorBuilder");
    }

    private PredominantColorProcessor checkPredominantColorProcessor(Object predominantColorProcessorArg) {
        return (PredominantColorProcessor) checkArg(predominantColorProcessorArg, PredominantColorProcessor.class, "predominantColorProcessor");
    }

    private PredominantColorProcessor.Swatch checkSwatch(String swatchString) {
        return (PredominantColorProcessor.Swatch) checkArg(swatchString, PredominantColorProcessor.Swatch.class, "swatch");
    }

    @JavascriptInterface
    @Block(classes = {PredominantColorProcessor.Builder.class}, constructor = true)
    public PredominantColorProcessor.Builder createPredominantColorProcessorBuilder() {
        try {
            startBlockExecution(BlockType.CREATE, "PredominantColorProcessor.Builder", "");
            return new PredominantColorProcessor.Builder();
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
    @Block(classes = {PredominantColorProcessor.Builder.class}, methodName = {"setRoi"})
    public void setRoi(Object predominantColorProcessorBuilderArg, Object imageRegionArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "PredominantColorProcessor.Builder", ".setRoi");
            PredominantColorProcessor.Builder predominantColorProcessorBuilder = checkPredominantColorProcessorBuilder(predominantColorProcessorBuilderArg);
            ImageRegion imageRegion = checkImageRegion(imageRegionArg);
            if (predominantColorProcessorBuilder != null && imageRegion != null) {
                predominantColorProcessorBuilder.setRoi(imageRegion);
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
    @Block(classes = {PredominantColorProcessor.Builder.class}, methodName = {"setSwatches"})
    public void setSwatches(Object predominantColorProcessorBuilderArg, String json) {
        try {
            startBlockExecution(BlockType.FUNCTION, "PredominantColorProcessor.Builder", ".setSwatches");
            PredominantColorProcessor.Builder predominantColorProcessorBuilder = checkPredominantColorProcessorBuilder(predominantColorProcessorBuilderArg);
            if (predominantColorProcessorBuilder != null) {
                String[] swatchStrings = (String[]) fromJson(json, String[].class);
                PredominantColorProcessor.Swatch[] swatches = new PredominantColorProcessor.Swatch[swatchStrings.length];
                for (int i = 0; i < swatches.length; i++) {
                    swatches[i] = checkSwatch(swatchStrings[i]);
                }
                predominantColorProcessorBuilder.setSwatches(swatches);
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
    @Block(classes = {PredominantColorProcessor.Builder.class}, methodName = {"build"})
    public PredominantColorProcessor buildPredominantColorProcessor(Object predominantColorProcessorBuilderArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "PredominantColorProcessor.Builder", ".build");
            PredominantColorProcessor.Builder predominantColorProcessorBuilder = checkPredominantColorProcessorBuilder(predominantColorProcessorBuilderArg);
            if (predominantColorProcessorBuilder != null) {
                return predominantColorProcessorBuilder.build();
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
    @Block(classes = {PredominantColorProcessor.class}, methodName = {"getAnalysis"})
    public String getAnalysis(Object predominantColorProcessorArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "PredominantColorProcessor", ".getAnalysis");
            PredominantColorProcessor predominantColorProcessor = checkPredominantColorProcessor(predominantColorProcessorArg);
            if (predominantColorProcessor != null) {
                return toJson(predominantColorProcessor.getAnalysis());
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
}
