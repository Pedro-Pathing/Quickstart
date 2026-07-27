package com.google.zxing.pdf417.encoder;

/* JADX INFO: loaded from: classes8.dex */
final class BarcodeRow {
    private int currentLocation = 0;
    private final byte[] row;

    BarcodeRow(int width) {
        this.row = new byte[width];
    }

    void set(int x, byte value) {
        this.row[x] = value;
    }

    private void set(int i, boolean z) {
        this.row[i] = z ? (byte) 1 : (byte) 0;
    }

    void addBar(boolean black, int width) {
        for (int ii = 0; ii < width; ii++) {
            int i = this.currentLocation;
            this.currentLocation = i + 1;
            set(i, black);
        }
    }

    byte[] getScaledRow(int scale) {
        byte[] output = new byte[this.row.length * scale];
        for (int i = 0; i < output.length; i++) {
            output[i] = this.row[i / scale];
        }
        return output;
    }
}
