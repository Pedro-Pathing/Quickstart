package com.google.zxing.pdf417.encoder;

/* JADX INFO: loaded from: classes8.dex */
public final class Dimensions {
    private final int maxCols;
    private final int maxRows;
    private final int minCols;
    private final int minRows;

    public Dimensions(int minCols, int maxCols, int minRows, int maxRows) {
        this.minCols = minCols;
        this.maxCols = maxCols;
        this.minRows = minRows;
        this.maxRows = maxRows;
    }

    public int getMinCols() {
        return this.minCols;
    }

    public int getMaxCols() {
        return this.maxCols;
    }

    public int getMinRows() {
        return this.minRows;
    }

    public int getMaxRows() {
        return this.maxRows;
    }
}
