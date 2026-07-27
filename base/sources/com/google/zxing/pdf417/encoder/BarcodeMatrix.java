package com.google.zxing.pdf417.encoder;

import java.lang.reflect.Array;

/* JADX INFO: loaded from: classes8.dex */
public final class BarcodeMatrix {
    private int currentRow;
    private final int height;
    private final BarcodeRow[] matrix;
    private final int width;

    BarcodeMatrix(int height, int width) {
        this.matrix = new BarcodeRow[height];
        int matrixLength = this.matrix.length;
        for (int i = 0; i < matrixLength; i++) {
            this.matrix[i] = new BarcodeRow(((width + 4) * 17) + 1);
        }
        int i2 = width * 17;
        this.width = i2;
        this.height = height;
        this.currentRow = -1;
    }

    void set(int x, int y, byte value) {
        this.matrix[y].set(x, value);
    }

    void startRow() {
        this.currentRow++;
    }

    BarcodeRow getCurrentRow() {
        return this.matrix[this.currentRow];
    }

    public byte[][] getMatrix() {
        return getScaledMatrix(1, 1);
    }

    public byte[][] getScaledMatrix(int xScale, int yScale) {
        byte[][] matrixOut = (byte[][]) Array.newInstance((Class<?>) Byte.TYPE, this.height * yScale, this.width * xScale);
        int yMax = this.height * yScale;
        for (int i = 0; i < yMax; i++) {
            matrixOut[(yMax - i) - 1] = this.matrix[i / yScale].getScaledRow(xScale);
        }
        return matrixOut;
    }
}
