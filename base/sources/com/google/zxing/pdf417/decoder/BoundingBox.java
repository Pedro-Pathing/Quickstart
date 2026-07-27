package com.google.zxing.pdf417.decoder;

import com.google.zxing.NotFoundException;
import com.google.zxing.ResultPoint;
import com.google.zxing.common.BitMatrix;

/* JADX INFO: loaded from: classes8.dex */
final class BoundingBox {
    private final ResultPoint bottomLeft;
    private final ResultPoint bottomRight;
    private final BitMatrix image;
    private final int maxX;
    private final int maxY;
    private final int minX;
    private final int minY;
    private final ResultPoint topLeft;
    private final ResultPoint topRight;

    BoundingBox(BitMatrix image, ResultPoint topLeft, ResultPoint bottomLeft, ResultPoint topRight, ResultPoint bottomRight) throws NotFoundException {
        boolean leftUnspecified = topLeft == null || bottomLeft == null;
        boolean rightUnspecified = topRight == null || bottomRight == null;
        if (leftUnspecified && rightUnspecified) {
            throw NotFoundException.getNotFoundInstance();
        }
        if (leftUnspecified) {
            topLeft = new ResultPoint(0.0f, topRight.getY());
            bottomLeft = new ResultPoint(0.0f, bottomRight.getY());
        } else if (rightUnspecified) {
            topRight = new ResultPoint(image.getWidth() - 1, topLeft.getY());
            bottomRight = new ResultPoint(image.getWidth() - 1, bottomLeft.getY());
        }
        this.image = image;
        this.topLeft = topLeft;
        this.bottomLeft = bottomLeft;
        this.topRight = topRight;
        this.bottomRight = bottomRight;
        this.minX = (int) Math.min(topLeft.getX(), bottomLeft.getX());
        this.maxX = (int) Math.max(topRight.getX(), bottomRight.getX());
        this.minY = (int) Math.min(topLeft.getY(), topRight.getY());
        this.maxY = (int) Math.max(bottomLeft.getY(), bottomRight.getY());
    }

    BoundingBox(BoundingBox boundingBox) {
        this.image = boundingBox.image;
        this.topLeft = boundingBox.topLeft;
        this.bottomLeft = boundingBox.bottomLeft;
        this.topRight = boundingBox.topRight;
        this.bottomRight = boundingBox.bottomRight;
        this.minX = boundingBox.minX;
        this.maxX = boundingBox.maxX;
        this.minY = boundingBox.minY;
        this.maxY = boundingBox.maxY;
    }

    static BoundingBox merge(BoundingBox leftBox, BoundingBox rightBox) throws NotFoundException {
        if (leftBox == null) {
            return rightBox;
        }
        if (rightBox == null) {
            return leftBox;
        }
        return new BoundingBox(leftBox.image, leftBox.topLeft, leftBox.bottomLeft, rightBox.topRight, rightBox.bottomRight);
    }

    BoundingBox addMissingRows(int i, int i2, boolean z) throws NotFoundException {
        ResultPoint resultPoint;
        ResultPoint resultPoint2;
        ResultPoint resultPoint3;
        ResultPoint resultPoint4;
        ResultPoint resultPoint5 = this.topLeft;
        ResultPoint resultPoint6 = this.bottomLeft;
        ResultPoint resultPoint7 = this.topRight;
        ResultPoint resultPoint8 = this.bottomRight;
        if (i <= 0) {
            resultPoint = resultPoint5;
            resultPoint2 = resultPoint7;
        } else {
            ResultPoint resultPoint9 = z ? this.topLeft : this.topRight;
            int y = ((int) resultPoint9.getY()) - i;
            if (y < 0) {
                y = 0;
            }
            ResultPoint resultPoint10 = new ResultPoint(resultPoint9.getX(), y);
            if (z) {
                resultPoint = resultPoint10;
                resultPoint2 = resultPoint7;
            } else {
                resultPoint2 = resultPoint10;
                resultPoint = resultPoint5;
            }
        }
        if (i2 <= 0) {
            resultPoint3 = resultPoint6;
            resultPoint4 = resultPoint8;
        } else {
            ResultPoint resultPoint11 = z ? this.bottomLeft : this.bottomRight;
            int y2 = ((int) resultPoint11.getY()) + i2;
            if (y2 >= this.image.getHeight()) {
                y2 = this.image.getHeight() - 1;
            }
            ResultPoint resultPoint12 = new ResultPoint(resultPoint11.getX(), y2);
            if (z) {
                resultPoint3 = resultPoint12;
                resultPoint4 = resultPoint8;
            } else {
                resultPoint4 = resultPoint12;
                resultPoint3 = resultPoint6;
            }
        }
        return new BoundingBox(this.image, resultPoint, resultPoint3, resultPoint2, resultPoint4);
    }

    int getMinX() {
        return this.minX;
    }

    int getMaxX() {
        return this.maxX;
    }

    int getMinY() {
        return this.minY;
    }

    int getMaxY() {
        return this.maxY;
    }

    ResultPoint getTopLeft() {
        return this.topLeft;
    }

    ResultPoint getTopRight() {
        return this.topRight;
    }

    ResultPoint getBottomLeft() {
        return this.bottomLeft;
    }

    ResultPoint getBottomRight() {
        return this.bottomRight;
    }
}
