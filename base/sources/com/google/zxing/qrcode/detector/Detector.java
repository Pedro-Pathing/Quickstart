package com.google.zxing.qrcode.detector;

import com.google.zxing.DecodeHintType;
import com.google.zxing.FormatException;
import com.google.zxing.NotFoundException;
import com.google.zxing.ResultPoint;
import com.google.zxing.ResultPointCallback;
import com.google.zxing.common.BitMatrix;
import com.google.zxing.common.DetectorResult;
import com.google.zxing.common.GridSampler;
import com.google.zxing.common.PerspectiveTransform;
import com.google.zxing.common.detector.MathUtils;
import com.google.zxing.qrcode.decoder.Version;
import java.util.Map;

/* JADX INFO: loaded from: classes8.dex */
public class Detector {
    private final BitMatrix image;
    private ResultPointCallback resultPointCallback;

    public Detector(BitMatrix image) {
        this.image = image;
    }

    protected final BitMatrix getImage() {
        return this.image;
    }

    protected final ResultPointCallback getResultPointCallback() {
        return this.resultPointCallback;
    }

    public DetectorResult detect() throws NotFoundException, FormatException {
        return detect(null);
    }

    public final DetectorResult detect(Map<DecodeHintType, ?> hints) throws NotFoundException, FormatException {
        this.resultPointCallback = hints == null ? null : (ResultPointCallback) hints.get(DecodeHintType.NEED_RESULT_POINT_CALLBACK);
        FinderPatternInfo info = new FinderPatternFinder(this.image, this.resultPointCallback).find(hints);
        return processFinderPatternInfo(info);
    }

    protected final DetectorResult processFinderPatternInfo(FinderPatternInfo info) throws NotFoundException, FormatException {
        ResultPoint[] points;
        FinderPattern topLeft = info.getTopLeft();
        FinderPattern topRight = info.getTopRight();
        FinderPattern bottomLeft = info.getBottomLeft();
        float moduleSize = calculateModuleSize(topLeft, topRight, bottomLeft);
        if (moduleSize < 1.0f) {
            throw NotFoundException.getNotFoundInstance();
        }
        int dimension = computeDimension(topLeft, topRight, bottomLeft, moduleSize);
        Version provisionalVersion = Version.getProvisionalVersionForDimension(dimension);
        int modulesBetweenFPCenters = provisionalVersion.getDimensionForVersion() - 7;
        AlignmentPattern alignmentPattern = null;
        if (provisionalVersion.getAlignmentPatternCenters().length > 0) {
            float bottomRightX = bottomLeft.getX() + (topRight.getX() - topLeft.getX());
            float bottomRightY = bottomLeft.getY() + (topRight.getY() - topLeft.getY());
            float correctionToTopLeft = 1.0f - (3.0f / modulesBetweenFPCenters);
            int estAlignmentX = (int) (topLeft.getX() + ((bottomRightX - topLeft.getX()) * correctionToTopLeft));
            int estAlignmentY = (int) (topLeft.getY() + ((bottomRightY - topLeft.getY()) * correctionToTopLeft));
            int i = 4;
            while (true) {
                if (i > 16) {
                    break;
                }
                try {
                    alignmentPattern = findAlignmentInRegion(moduleSize, estAlignmentX, estAlignmentY, i);
                    break;
                } catch (NotFoundException e) {
                    i <<= 1;
                }
            }
        }
        PerspectiveTransform transform = createTransform(topLeft, topRight, bottomLeft, alignmentPattern, dimension);
        BitMatrix bits = sampleGrid(this.image, transform, dimension);
        if (alignmentPattern == null) {
            points = new ResultPoint[]{bottomLeft, topLeft, topRight};
        } else {
            points = new ResultPoint[]{bottomLeft, topLeft, topRight, alignmentPattern};
        }
        return new DetectorResult(bits, points);
    }

    private static PerspectiveTransform createTransform(ResultPoint topLeft, ResultPoint topRight, ResultPoint bottomLeft, ResultPoint alignmentPattern, int dimension) {
        float bottomRightX;
        float bottomRightY;
        float sourceBottomRightY;
        float sourceBottomRightX;
        float dimMinusThree = dimension - 3.5f;
        if (alignmentPattern != null) {
            bottomRightX = alignmentPattern.getX();
            float bottomRightY2 = alignmentPattern.getY();
            float sourceBottomRightX2 = dimMinusThree - 3.0f;
            bottomRightY = bottomRightY2;
            sourceBottomRightY = sourceBottomRightX2;
            sourceBottomRightX = sourceBottomRightX2;
        } else {
            float bottomRightX2 = topRight.getX();
            bottomRightX = (bottomRightX2 - topLeft.getX()) + bottomLeft.getX();
            float bottomRightY3 = (topRight.getY() - topLeft.getY()) + bottomLeft.getY();
            bottomRightY = bottomRightY3;
            sourceBottomRightY = dimMinusThree;
            sourceBottomRightX = dimMinusThree;
        }
        return PerspectiveTransform.quadrilateralToQuadrilateral(3.5f, 3.5f, dimMinusThree, 3.5f, sourceBottomRightX, sourceBottomRightY, 3.5f, dimMinusThree, topLeft.getX(), topLeft.getY(), topRight.getX(), topRight.getY(), bottomRightX, bottomRightY, bottomLeft.getX(), bottomLeft.getY());
    }

    private static BitMatrix sampleGrid(BitMatrix image, PerspectiveTransform transform, int dimension) throws NotFoundException {
        return GridSampler.getInstance().sampleGrid(image, dimension, dimension, transform);
    }

    private static int computeDimension(ResultPoint topLeft, ResultPoint topRight, ResultPoint bottomLeft, float moduleSize) throws NotFoundException {
        int tltrCentersDimension = MathUtils.round(ResultPoint.distance(topLeft, topRight) / moduleSize);
        int tlblCentersDimension = MathUtils.round(ResultPoint.distance(topLeft, bottomLeft) / moduleSize);
        int dimension = ((tltrCentersDimension + tlblCentersDimension) / 2) + 7;
        switch (dimension & 3) {
            case 0:
                return dimension + 1;
            case 1:
            default:
                return dimension;
            case 2:
                return dimension - 1;
            case 3:
                throw NotFoundException.getNotFoundInstance();
        }
    }

    protected final float calculateModuleSize(ResultPoint topLeft, ResultPoint topRight, ResultPoint bottomLeft) {
        return (calculateModuleSizeOneWay(topLeft, topRight) + calculateModuleSizeOneWay(topLeft, bottomLeft)) / 2.0f;
    }

    private float calculateModuleSizeOneWay(ResultPoint pattern, ResultPoint otherPattern) {
        float moduleSizeEst1 = sizeOfBlackWhiteBlackRunBothWays((int) pattern.getX(), (int) pattern.getY(), (int) otherPattern.getX(), (int) otherPattern.getY());
        float moduleSizeEst2 = sizeOfBlackWhiteBlackRunBothWays((int) otherPattern.getX(), (int) otherPattern.getY(), (int) pattern.getX(), (int) pattern.getY());
        if (Float.isNaN(moduleSizeEst1)) {
            return moduleSizeEst2 / 7.0f;
        }
        if (Float.isNaN(moduleSizeEst2)) {
            return moduleSizeEst1 / 7.0f;
        }
        return (moduleSizeEst1 + moduleSizeEst2) / 14.0f;
    }

    private float sizeOfBlackWhiteBlackRunBothWays(int fromX, int fromY, int toX, int toY) {
        float result = sizeOfBlackWhiteBlackRun(fromX, fromY, toX, toY);
        float scale = 1.0f;
        int i = fromX - (toX - fromX);
        int otherToX = i;
        if (i < 0) {
            scale = fromX / (fromX - otherToX);
            otherToX = 0;
        } else if (otherToX >= this.image.getWidth()) {
            scale = ((this.image.getWidth() - 1) - fromX) / (otherToX - fromX);
            otherToX = this.image.getWidth() - 1;
        }
        int otherToY = (int) (fromY - ((toY - fromY) * scale));
        float scale2 = 1.0f;
        if (otherToY < 0) {
            scale2 = fromY / (fromY - otherToY);
            otherToY = 0;
        } else if (otherToY >= this.image.getHeight()) {
            scale2 = ((this.image.getHeight() - 1) - fromY) / (otherToY - fromY);
            otherToY = this.image.getHeight() - 1;
        }
        return (sizeOfBlackWhiteBlackRun(fromX, fromY, (int) (fromX + ((otherToX - fromX) * scale2)), otherToY) + result) - 1.0f;
    }

    private float sizeOfBlackWhiteBlackRun(int fromX, int fromY, int toX, int toY) {
        int fromX2;
        int fromY2;
        int toX2;
        int toY2;
        int i = 1;
        boolean z = Math.abs(toY - fromY) > Math.abs(toX - fromX);
        boolean steep = z;
        if (!z) {
            fromX2 = fromX;
            fromY2 = fromY;
            toX2 = toX;
            toY2 = toY;
        } else {
            fromX2 = fromY;
            fromY2 = fromX;
            toX2 = toY;
            toY2 = toX;
        }
        int dx = Math.abs(toX2 - fromX2);
        int dy = Math.abs(toY2 - fromY2);
        int error = (-dx) / 2;
        int xstep = fromX2 < toX2 ? 1 : -1;
        int ystep = fromY2 < toY2 ? 1 : -1;
        int state = 0;
        int xLimit = toX2 + xstep;
        int x = fromX2;
        int y = fromY2;
        while (x != xLimit) {
            int realX = steep ? y : x;
            int realY = steep ? x : y;
            if (state != i) {
                i = 0;
            }
            boolean steep2 = steep;
            int realX2 = xLimit;
            if (i == this.image.get(realX, realY)) {
                if (state == 2) {
                    return MathUtils.distance(x, y, fromX2, fromY2);
                }
                state++;
            }
            int i2 = error + dy;
            error = i2;
            if (i2 > 0) {
                if (y == toY2) {
                    break;
                }
                y += ystep;
                error -= dx;
            }
            x += xstep;
            xLimit = realX2;
            steep = steep2;
            i = 1;
        }
        if (state == 2) {
            return MathUtils.distance(toX2 + xstep, toY2, fromX2, fromY2);
        }
        return Float.NaN;
    }

    protected final AlignmentPattern findAlignmentInRegion(float overallEstModuleSize, int estAlignmentX, int estAlignmentY, float allowanceFactor) throws NotFoundException {
        int allowance = (int) (allowanceFactor * overallEstModuleSize);
        int alignmentAreaLeftX = Math.max(0, estAlignmentX - allowance);
        int alignmentAreaRightX = Math.min(this.image.getWidth() - 1, estAlignmentX + allowance);
        if (alignmentAreaRightX - alignmentAreaLeftX < overallEstModuleSize * 3.0f) {
            throw NotFoundException.getNotFoundInstance();
        }
        int alignmentAreaTopY = Math.max(0, estAlignmentY - allowance);
        int alignmentAreaBottomY = Math.min(this.image.getHeight() - 1, estAlignmentY + allowance);
        if (alignmentAreaBottomY - alignmentAreaTopY < overallEstModuleSize * 3.0f) {
            throw NotFoundException.getNotFoundInstance();
        }
        return new AlignmentPatternFinder(this.image, alignmentAreaLeftX, alignmentAreaTopY, alignmentAreaRightX - alignmentAreaLeftX, alignmentAreaBottomY - alignmentAreaTopY, overallEstModuleSize, this.resultPointCallback).find();
    }
}
