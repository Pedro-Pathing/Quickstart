package com.google.zxing;

import com.google.zxing.common.detector.MathUtils;

/* JADX INFO: loaded from: classes8.dex */
public class ResultPoint {
    private final float x;
    private final float y;

    public ResultPoint(float x, float y) {
        this.x = x;
        this.y = y;
    }

    public final float getX() {
        return this.x;
    }

    public final float getY() {
        return this.y;
    }

    public final boolean equals(Object other) {
        if (!(other instanceof ResultPoint)) {
            return false;
        }
        ResultPoint otherPoint = (ResultPoint) other;
        return this.x == otherPoint.x && this.y == otherPoint.y;
    }

    public final int hashCode() {
        return (Float.floatToIntBits(this.x) * 31) + Float.floatToIntBits(this.y);
    }

    public final String toString() {
        return "(" + this.x + ',' + this.y + ')';
    }

    public static void orderBestPatterns(ResultPoint[] patterns) {
        ResultPoint pointB;
        ResultPoint pointA;
        ResultPoint pointC;
        float zeroOneDistance = distance(patterns[0], patterns[1]);
        float oneTwoDistance = distance(patterns[1], patterns[2]);
        float zeroTwoDistance = distance(patterns[0], patterns[2]);
        if (oneTwoDistance >= zeroOneDistance && oneTwoDistance >= zeroTwoDistance) {
            pointB = patterns[0];
            pointA = patterns[1];
            pointC = patterns[2];
        } else if (zeroTwoDistance >= oneTwoDistance && zeroTwoDistance >= zeroOneDistance) {
            pointB = patterns[1];
            pointA = patterns[0];
            pointC = patterns[2];
        } else {
            pointB = patterns[2];
            pointA = patterns[0];
            pointC = patterns[1];
        }
        if (crossProductZ(pointA, pointB, pointC) < 0.0f) {
            ResultPoint temp = pointA;
            pointA = pointC;
            pointC = temp;
        }
        patterns[0] = pointA;
        patterns[1] = pointB;
        patterns[2] = pointC;
    }

    public static float distance(ResultPoint pattern1, ResultPoint pattern2) {
        return MathUtils.distance(pattern1.x, pattern1.y, pattern2.x, pattern2.y);
    }

    private static float crossProductZ(ResultPoint pointA, ResultPoint pointB, ResultPoint pointC) {
        float bX = pointB.x;
        float bY = pointB.y;
        return ((pointC.x - bX) * (pointA.y - bY)) - ((pointC.y - bY) * (pointA.x - bX));
    }
}
