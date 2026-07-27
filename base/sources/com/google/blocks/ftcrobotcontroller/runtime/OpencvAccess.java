package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Scalar;

/* JADX INFO: loaded from: classes8.dex */
class OpencvAccess extends Access {
    OpencvAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "");
    }

    private ColorSpace checkColorSpace(String colorSpaceString) {
        return (ColorSpace) checkArg(colorSpaceString, ColorSpace.class, "colorSpace");
    }

    private Scalar checkScalar(Object scalarArg, String socketName) {
        return (Scalar) checkArg(scalarArg, Scalar.class, socketName);
    }

    @JavascriptInterface
    @Block(classes = {ColorRange.class}, fieldName = {"ARTIFACT_GREEN", "ARTIFACT_PURPLE", "BLUE", "RED", "YELLOW", "GREEN"})
    public ColorRange colorRange(String color) {
        try {
            startBlockExecution(BlockType.GETTER, "ColorRange", "." + color);
            if (color.equals("ARTIFACT_GREEN")) {
                return ColorRange.ARTIFACT_GREEN;
            }
            if (color.equals("ARTIFACT_PURPLE")) {
                return ColorRange.ARTIFACT_PURPLE;
            }
            if (color.equals("BLUE")) {
                return ColorRange.BLUE;
            }
            if (color.equals("RED")) {
                return ColorRange.RED;
            }
            if (color.equals("YELLOW")) {
                return ColorRange.YELLOW;
            }
            if (color.equals("GREEN")) {
                return ColorRange.GREEN;
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
    @Block(classes = {ColorRange.class}, constructor = true)
    public ColorRange createColorRange(String colorSpaceString, Object minArg, Object maxArg) {
        try {
            startBlockExecution(BlockType.CREATE, "ColorRange", "");
            ColorSpace colorSpace = checkColorSpace(colorSpaceString);
            Scalar min = checkScalar(minArg, "min");
            Scalar max = checkScalar(maxArg, "max");
            if (colorSpace != null && min != null && max != null) {
                return new ColorRange(colorSpace, min, max);
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
    @Block(classes = {ImageRegion.class}, methodName = {"asImageCoordinates"})
    public ImageRegion asImageCoordinates(int left, int top, int right, int bottom) {
        try {
            startBlockExecution(BlockType.FUNCTION, "ImageRegion", ".asImageCoordinates");
            return ImageRegion.asImageCoordinates(left, top, right, bottom);
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
    @Block(classes = {ImageRegion.class}, methodName = {"asUnityCenterCoordinates"})
    public ImageRegion asUnityCenterCoordinates(double left, double top, double right, double bottom) {
        try {
            startBlockExecution(BlockType.FUNCTION, "ImageRegion", ".asUnityCenterCoordinates");
            return ImageRegion.asUnityCenterCoordinates(left, top, right, bottom);
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
    @Block(classes = {ImageRegion.class}, methodName = {"entireFrame"})
    public ImageRegion entireFrame() {
        try {
            startBlockExecution(BlockType.FUNCTION, "ImageRegion", ".entireFrame");
            return ImageRegion.entireFrame();
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
    @Block(classes = {Scalar.class}, constructor = true)
    public Scalar createScalar_with3(double v0, double v1, double v2) {
        try {
            startBlockExecution(BlockType.CREATE, "Scalar", "");
            return new Scalar(v0, v1, v2);
        } finally {
        }
    }
}
