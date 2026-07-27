package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.SortOrder;
import com.sun.tools.doclint.DocLint;
import java.util.List;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

/* JADX INFO: loaded from: classes8.dex */
class ColorBlobLocatorAccess extends Access {
    ColorBlobLocatorAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "");
    }

    private ColorBlobLocatorProcessor.Builder checkColorBlobLocatorProcessorBuilder(Object colorBlobLocatorProcessorBuilderArg) {
        return (ColorBlobLocatorProcessor.Builder) checkArg(colorBlobLocatorProcessorBuilderArg, ColorBlobLocatorProcessor.Builder.class, "colorBlobLocatorProcessorBuilder");
    }

    private ColorRange checkColorRange(Object colorRangeArg) {
        return (ColorRange) checkArg(colorRangeArg, ColorRange.class, "colorRange");
    }

    private ColorBlobLocatorProcessor.ContourMode checkContourMode(String contourModeString) {
        return (ColorBlobLocatorProcessor.ContourMode) checkArg(contourModeString, ColorBlobLocatorProcessor.ContourMode.class, "contourMode");
    }

    private ColorBlobLocatorProcessor.MorphOperationType checkMorphOperationType(String morphOperationTypeString) {
        return (ColorBlobLocatorProcessor.MorphOperationType) checkArg(morphOperationTypeString, ColorBlobLocatorProcessor.MorphOperationType.class, "morphOperationType");
    }

    private ColorBlobLocatorProcessor checkColorBlobLocatorProcessor(Object colorBlobLocatorProcessorArg) {
        return (ColorBlobLocatorProcessor) checkArg(colorBlobLocatorProcessorArg, ColorBlobLocatorProcessor.class, "colorBlobLocatorProcessor");
    }

    private ColorBlobLocatorProcessor.BlobCriteria checkBlobCriteria(String criteriaString) {
        return (ColorBlobLocatorProcessor.BlobCriteria) checkArg(criteriaString, ColorBlobLocatorProcessor.BlobCriteria.class, "criteria");
    }

    private ColorBlobLocatorProcessor.BlobFilter checkBlobFilter(Object filterArg) {
        return (ColorBlobLocatorProcessor.BlobFilter) checkArg(filterArg, ColorBlobLocatorProcessor.BlobFilter.class, "filter");
    }

    private ColorBlobLocatorProcessor.BlobSort checkBlobSort(Object sortArg) {
        return (ColorBlobLocatorProcessor.BlobSort) checkArg(sortArg, ColorBlobLocatorProcessor.BlobSort.class, "sort");
    }

    private SortOrder checkSortOrder(String sortOrderString) {
        return (SortOrder) checkArg(sortOrderString, SortOrder.class, "sortOrder");
    }

    @JavascriptInterface
    @Block(classes = {ColorBlobLocatorProcessor.Builder.class}, constructor = true)
    public ColorBlobLocatorProcessor.Builder createColorBlobLocatorProcessorBuilder() {
        try {
            startBlockExecution(BlockType.CREATE, "ColorBlobLocatorProcessor.Builder", "");
            return new ColorBlobLocatorProcessor.Builder();
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
    @Block(classes = {ColorBlobLocatorProcessor.Builder.class}, methodName = {"setDrawContours"})
    public void setDrawContours(Object colorBlobLocatorProcessorBuilderArg, boolean drawContours) {
        try {
            startBlockExecution(BlockType.FUNCTION, "ColorBlobLocatorProcessor.Builder", ".setDrawContours");
            ColorBlobLocatorProcessor.Builder colorBlobLocatorProcessorBuilder = checkColorBlobLocatorProcessorBuilder(colorBlobLocatorProcessorBuilderArg);
            if (colorBlobLocatorProcessorBuilder != null) {
                colorBlobLocatorProcessorBuilder.setDrawContours(drawContours);
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
    @Block(classes = {ColorBlobLocatorProcessor.Builder.class}, methodName = {"setBoxFitColor"})
    public void setBoxFitColor(Object colorBlobLocatorProcessorBuilderArg, int color) {
        try {
            startBlockExecution(BlockType.FUNCTION, "ColorBlobLocatorProcessor.Builder", ".setBoxFitColor");
            ColorBlobLocatorProcessor.Builder colorBlobLocatorProcessorBuilder = checkColorBlobLocatorProcessorBuilder(colorBlobLocatorProcessorBuilderArg);
            if (colorBlobLocatorProcessorBuilder != null) {
                colorBlobLocatorProcessorBuilder.setBoxFitColor(color);
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
    @Block(classes = {ColorBlobLocatorProcessor.Builder.class}, methodName = {"setCircleFitColor"})
    public void setCircleFitColor(Object colorBlobLocatorProcessorBuilderArg, int color) {
        try {
            startBlockExecution(BlockType.FUNCTION, "ColorBlobLocatorProcessor.Builder", ".setCircleFitColor");
            ColorBlobLocatorProcessor.Builder colorBlobLocatorProcessorBuilder = checkColorBlobLocatorProcessorBuilder(colorBlobLocatorProcessorBuilderArg);
            if (colorBlobLocatorProcessorBuilder != null) {
                colorBlobLocatorProcessorBuilder.setCircleFitColor(color);
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
    @Block(classes = {ColorBlobLocatorProcessor.Builder.class}, methodName = {"setRoiColor"})
    public void setRoiColor(Object colorBlobLocatorProcessorBuilderArg, int color) {
        try {
            startBlockExecution(BlockType.FUNCTION, "ColorBlobLocatorProcessor.Builder", ".setRoiColor");
            ColorBlobLocatorProcessor.Builder colorBlobLocatorProcessorBuilder = checkColorBlobLocatorProcessorBuilder(colorBlobLocatorProcessorBuilderArg);
            if (colorBlobLocatorProcessorBuilder != null) {
                colorBlobLocatorProcessorBuilder.setRoiColor(color);
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
    @Block(classes = {ColorBlobLocatorProcessor.Builder.class}, methodName = {"setContourColor"})
    public void setContourColor(Object colorBlobLocatorProcessorBuilderArg, int color) {
        try {
            startBlockExecution(BlockType.FUNCTION, "ColorBlobLocatorProcessor.Builder", ".setContourColor");
            ColorBlobLocatorProcessor.Builder colorBlobLocatorProcessorBuilder = checkColorBlobLocatorProcessorBuilder(colorBlobLocatorProcessorBuilderArg);
            if (colorBlobLocatorProcessorBuilder != null) {
                colorBlobLocatorProcessorBuilder.setContourColor(color);
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
    @Block(classes = {ColorBlobLocatorProcessor.Builder.class}, methodName = {"setTargetColorRange"})
    public void setTargetColorRange(Object colorBlobLocatorProcessorBuilderArg, Object colorRangeArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "ColorBlobLocatorProcessor.Builder", ".setTargetColorRange");
            ColorBlobLocatorProcessor.Builder colorBlobLocatorProcessorBuilder = checkColorBlobLocatorProcessorBuilder(colorBlobLocatorProcessorBuilderArg);
            ColorRange colorRange = checkColorRange(colorRangeArg);
            if (colorBlobLocatorProcessorBuilder != null && colorRange != null) {
                colorBlobLocatorProcessorBuilder.setTargetColorRange(colorRange);
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
    @Block(classes = {ColorBlobLocatorProcessor.Builder.class}, methodName = {"setContourMode"})
    public void setContourMode(Object colorBlobLocatorProcessorBuilderArg, String contourModeString) {
        try {
            startBlockExecution(BlockType.FUNCTION, "ColorBlobLocatorProcessor.Builder", ".setContourMode");
            ColorBlobLocatorProcessor.Builder colorBlobLocatorProcessorBuilder = checkColorBlobLocatorProcessorBuilder(colorBlobLocatorProcessorBuilderArg);
            ColorBlobLocatorProcessor.ContourMode contourMode = checkContourMode(contourModeString);
            if (colorBlobLocatorProcessorBuilder != null && contourMode != null) {
                colorBlobLocatorProcessorBuilder.setContourMode(contourMode);
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
    @Block(classes = {ColorBlobLocatorProcessor.Builder.class}, methodName = {"setRoi"})
    public void setRoi(Object colorBlobLocatorProcessorBuilderArg, Object imageRegionArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "ColorBlobLocatorProcessor.Builder", ".setRoi");
            ColorBlobLocatorProcessor.Builder colorBlobLocatorProcessorBuilder = checkColorBlobLocatorProcessorBuilder(colorBlobLocatorProcessorBuilderArg);
            ImageRegion imageRegion = checkImageRegion(imageRegionArg);
            if (colorBlobLocatorProcessorBuilder != null && imageRegion != null) {
                colorBlobLocatorProcessorBuilder.setRoi(imageRegion);
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
    @Block(classes = {ColorBlobLocatorProcessor.Builder.class}, methodName = {"setBlurSize"})
    public void setBlurSize(Object colorBlobLocatorProcessorBuilderArg, int blurSize) {
        try {
            startBlockExecution(BlockType.FUNCTION, "ColorBlobLocatorProcessor.Builder", ".setBlurSize");
            ColorBlobLocatorProcessor.Builder colorBlobLocatorProcessorBuilder = checkColorBlobLocatorProcessorBuilder(colorBlobLocatorProcessorBuilderArg);
            if (colorBlobLocatorProcessorBuilder != null) {
                colorBlobLocatorProcessorBuilder.setBlurSize(blurSize);
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
    @Block(classes = {ColorBlobLocatorProcessor.Builder.class}, methodName = {"setMorphOperationType"})
    public void setMorphOperationType(Object colorBlobLocatorProcessorBuilderArg, String morphOperationTypeString) {
        try {
            startBlockExecution(BlockType.FUNCTION, "ColorBlobLocatorProcessor.Builder", ".setMorphOperationType");
            ColorBlobLocatorProcessor.Builder colorBlobLocatorProcessorBuilder = checkColorBlobLocatorProcessorBuilder(colorBlobLocatorProcessorBuilderArg);
            ColorBlobLocatorProcessor.MorphOperationType morphOperationType = checkMorphOperationType(morphOperationTypeString);
            if (colorBlobLocatorProcessorBuilder != null && morphOperationType != null) {
                colorBlobLocatorProcessorBuilder.setMorphOperationType(morphOperationType);
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
    @Block(classes = {ColorBlobLocatorProcessor.Builder.class}, methodName = {"setErodeSize"})
    public void setErodeSize(Object colorBlobLocatorProcessorBuilderArg, int erodeSize) {
        try {
            startBlockExecution(BlockType.FUNCTION, "ColorBlobLocatorProcessor.Builder", ".setErodeSize");
            ColorBlobLocatorProcessor.Builder colorBlobLocatorProcessorBuilder = checkColorBlobLocatorProcessorBuilder(colorBlobLocatorProcessorBuilderArg);
            if (colorBlobLocatorProcessorBuilder != null) {
                colorBlobLocatorProcessorBuilder.setErodeSize(erodeSize);
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
    @Block(classes = {ColorBlobLocatorProcessor.Builder.class}, methodName = {"setDilateSize"})
    public void setDilateSize(Object colorBlobLocatorProcessorBuilderArg, int dilateSize) {
        try {
            startBlockExecution(BlockType.FUNCTION, "ColorBlobLocatorProcessor.Builder", ".setDilateSize");
            ColorBlobLocatorProcessor.Builder colorBlobLocatorProcessorBuilder = checkColorBlobLocatorProcessorBuilder(colorBlobLocatorProcessorBuilderArg);
            if (colorBlobLocatorProcessorBuilder != null) {
                colorBlobLocatorProcessorBuilder.setDilateSize(dilateSize);
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
    @Block(classes = {ColorBlobLocatorProcessor.Builder.class}, methodName = {"build"})
    public ColorBlobLocatorProcessor buildColorBlobLocatorProcessor(Object colorBlobLocatorProcessorBuilderArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "ColorBlobLocatorProcessor.Builder", ".build");
            ColorBlobLocatorProcessor.Builder colorBlobLocatorProcessorBuilder = checkColorBlobLocatorProcessorBuilder(colorBlobLocatorProcessorBuilderArg);
            if (colorBlobLocatorProcessorBuilder != null) {
                return colorBlobLocatorProcessorBuilder.build();
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
    @Block(classes = {ColorBlobLocatorProcessor.BlobFilter.class}, constructor = true)
    public ColorBlobLocatorProcessor.BlobFilter createColorBlobLocatorProcessorBlobFilter(String criteriaString, double minValue, double maxValue) {
        try {
            startBlockExecution(BlockType.CREATE, "");
            ColorBlobLocatorProcessor.BlobCriteria criteria = checkBlobCriteria(criteriaString);
            if (criteria != null) {
                return new ColorBlobLocatorProcessor.BlobFilter(criteria, minValue, maxValue);
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
    @Block(classes = {ColorBlobLocatorProcessor.class}, methodName = {"addFilter"})
    public void addFilter(Object colorBlobLocatorProcessorArg, Object filterArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "ColorBlobLocatorProcessor", ".addFilter");
            ColorBlobLocatorProcessor colorBlobLocatorProcessor = checkColorBlobLocatorProcessor(colorBlobLocatorProcessorArg);
            ColorBlobLocatorProcessor.BlobFilter filter = checkBlobFilter(filterArg);
            if (colorBlobLocatorProcessor != null && filter != null) {
                colorBlobLocatorProcessor.addFilter(filter);
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
    @Block(classes = {ColorBlobLocatorProcessor.class}, methodName = {"removeFilter"})
    public void removeFilter(Object colorBlobLocatorProcessorArg, Object filterArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "ColorBlobLocatorProcessor", ".removeFilter");
            ColorBlobLocatorProcessor colorBlobLocatorProcessor = checkColorBlobLocatorProcessor(colorBlobLocatorProcessorArg);
            ColorBlobLocatorProcessor.BlobFilter filter = checkBlobFilter(filterArg);
            if (colorBlobLocatorProcessor != null && filter != null) {
                colorBlobLocatorProcessor.removeFilter(filter);
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
    @Block(classes = {ColorBlobLocatorProcessor.class}, methodName = {"removeAllFilters"})
    public void removeAllFilters(Object colorBlobLocatorProcessorArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "ColorBlobLocatorProcessor", ".removeAllFilters");
            ColorBlobLocatorProcessor colorBlobLocatorProcessor = checkColorBlobLocatorProcessor(colorBlobLocatorProcessorArg);
            if (colorBlobLocatorProcessor != null) {
                colorBlobLocatorProcessor.removeAllFilters();
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
    @Block(classes = {ColorBlobLocatorProcessor.BlobSort.class}, constructor = true)
    public ColorBlobLocatorProcessor.BlobSort createColorBlobLocatorProcessorBlobSort(String criteriaString, String sortOrderString) {
        try {
            startBlockExecution(BlockType.CREATE, "");
            ColorBlobLocatorProcessor.BlobCriteria criteria = checkBlobCriteria(criteriaString);
            SortOrder sortOrder = checkSortOrder(sortOrderString);
            if (criteria != null && sortOrder != null) {
                return new ColorBlobLocatorProcessor.BlobSort(criteria, sortOrder);
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
    @Block(classes = {ColorBlobLocatorProcessor.class}, methodName = {"setSort"})
    public void setSort(Object colorBlobLocatorProcessorArg, Object sortArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "ColorBlobLocatorProcessor", ".setSort");
            ColorBlobLocatorProcessor colorBlobLocatorProcessor = checkColorBlobLocatorProcessor(colorBlobLocatorProcessorArg);
            ColorBlobLocatorProcessor.BlobSort sort = checkBlobSort(sortArg);
            if (colorBlobLocatorProcessor != null && sort != null) {
                colorBlobLocatorProcessor.setSort(sort);
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
    @Block(classes = {ColorBlobLocatorProcessor.class}, methodName = {"getBlobs"})
    public String getBlobs(Object colorBlobLocatorProcessorArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "ColorBlobLocatorProcessor", ".getBlobs");
            ColorBlobLocatorProcessor colorBlobLocatorProcessor = checkColorBlobLocatorProcessor(colorBlobLocatorProcessorArg);
            if (colorBlobLocatorProcessor != null) {
                return blobsToJson(colorBlobLocatorProcessor.getBlobs());
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

    private static String blobsToJson(List<ColorBlobLocatorProcessor.Blob> blobs) {
        StringBuilder json = new StringBuilder();
        json.append("[");
        String delimiter = "";
        for (ColorBlobLocatorProcessor.Blob blob : blobs) {
            double density = LynxServoController.apiPositionFirst;
            try {
                density = blob.getDensity();
            } catch (Exception e) {
            }
            double aspectRatio = LynxServoController.apiPositionFirst;
            try {
                aspectRatio = blob.getAspectRatio();
            } catch (Exception e2) {
            }
            json.append(delimiter).append("{").append("\"ContourPoints\":").append(toJson(blob.getContourPoints())).append(DocLint.TAGS_SEPARATOR).append("\"ContourArea\":").append(blob.getContourArea()).append(DocLint.TAGS_SEPARATOR).append("\"Density\":").append(fixDouble(density)).append(DocLint.TAGS_SEPARATOR).append("\"AspectRatio\":").append(fixDouble(aspectRatio)).append(DocLint.TAGS_SEPARATOR).append("\"BoxFit\":").append(rotatedRectToJson(blob.getBoxFit())).append(DocLint.TAGS_SEPARATOR).append("\"ArcLength\":").append(fixDouble(blob.getArcLength())).append(DocLint.TAGS_SEPARATOR).append("\"Circularity\":").append(fixDouble(blob.getCircularity())).append(DocLint.TAGS_SEPARATOR).append("\"Circle\":").append(circleToJson(blob.getCircle())).append("}");
            delimiter = DocLint.TAGS_SEPARATOR;
        }
        json.append("]");
        return json.toString();
    }

    private static String rotatedRectToJson(RotatedRect rotatedRect) {
        String originalJson = toJson(rotatedRect);
        if (!originalJson.endsWith("}")) {
            RobotLog.ww("ColorBlobLocatorAccess", "Unexpected: result from toJson(RotatedRect) does not end with '}'!");
            return originalJson;
        }
        Point[] points = new Point[4];
        rotatedRect.points(points);
        String json = originalJson.substring(0, originalJson.length() - 1) + DocLint.TAGS_SEPARATOR + "\"boundingRect\":" + toJson(rotatedRect.boundingRect()) + DocLint.TAGS_SEPARATOR + "\"points\":" + toJson(points) + "}";
        return json;
    }

    private static String circleToJson(Circle circle) {
        String json = "{\"Center\":" + toJson(circle.getCenter()) + DocLint.TAGS_SEPARATOR + "\"Radius\":" + circle.getRadius() + DocLint.TAGS_SEPARATOR + "\"X\":" + circle.getX() + DocLint.TAGS_SEPARATOR + "\"Y\":" + circle.getY() + "}";
        return json;
    }

    private static double fixDouble(double v) {
        if (Double.isFinite(v)) {
            return v;
        }
        return LynxServoController.apiPositionFirst;
    }
}
