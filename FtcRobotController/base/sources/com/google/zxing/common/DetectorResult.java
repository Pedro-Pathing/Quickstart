package com.google.zxing.common;

import com.google.zxing.ResultPoint;

/* JADX INFO: loaded from: classes8.dex */
public class DetectorResult {
    private final BitMatrix bits;
    private final ResultPoint[] points;

    public DetectorResult(BitMatrix bits, ResultPoint[] points) {
        this.bits = bits;
        this.points = points;
    }

    public final BitMatrix getBits() {
        return this.bits;
    }

    public final ResultPoint[] getPoints() {
        return this.points;
    }
}
