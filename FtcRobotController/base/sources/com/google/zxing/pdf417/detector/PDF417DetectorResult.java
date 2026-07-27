package com.google.zxing.pdf417.detector;

import com.google.zxing.ResultPoint;
import com.google.zxing.common.BitMatrix;
import java.util.List;

/* JADX INFO: loaded from: classes8.dex */
public final class PDF417DetectorResult {
    private final BitMatrix bits;
    private final List<ResultPoint[]> points;

    public PDF417DetectorResult(BitMatrix bits, List<ResultPoint[]> points) {
        this.bits = bits;
        this.points = points;
    }

    public BitMatrix getBits() {
        return this.bits;
    }

    public List<ResultPoint[]> getPoints() {
        return this.points;
    }
}
