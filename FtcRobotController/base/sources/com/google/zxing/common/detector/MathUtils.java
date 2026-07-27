package com.google.zxing.common.detector;

/* JADX INFO: loaded from: classes8.dex */
public final class MathUtils {
    private MathUtils() {
    }

    public static int round(float d) {
        return (int) ((d < 0.0f ? -0.5f : 0.5f) + d);
    }

    public static float distance(float aX, float aY, float bX, float bY) {
        double xDiff = aX - bX;
        double yDiff = aY - bY;
        return (float) Math.sqrt((xDiff * xDiff) + (yDiff * yDiff));
    }

    public static float distance(int aX, int aY, int bX, int bY) {
        double xDiff = aX - bX;
        double yDiff = aY - bY;
        return (float) Math.sqrt((xDiff * xDiff) + (yDiff * yDiff));
    }

    public static int sum(int[] array) {
        int count = 0;
        for (int a : array) {
            count += a;
        }
        return count;
    }
}
