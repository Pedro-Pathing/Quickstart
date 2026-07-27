package com.google.zxing;

import androidx.core.view.ViewCompat;

/* JADX INFO: loaded from: classes8.dex */
public final class PlanarYUVLuminanceSource extends LuminanceSource {
    private static final int THUMBNAIL_SCALE_FACTOR = 2;
    private final int dataHeight;
    private final int dataWidth;
    private final int left;
    private final int top;
    private final byte[] yuvData;

    public PlanarYUVLuminanceSource(byte[] yuvData, int dataWidth, int dataHeight, int left, int top, int width, int height, boolean reverseHorizontal) {
        super(width, height);
        if (left + width > dataWidth || top + height > dataHeight) {
            throw new IllegalArgumentException("Crop rectangle does not fit within image data.");
        }
        this.yuvData = yuvData;
        this.dataWidth = dataWidth;
        this.dataHeight = dataHeight;
        this.left = left;
        this.top = top;
        if (reverseHorizontal) {
            reverseHorizontal(width, height);
        }
    }

    @Override // com.google.zxing.LuminanceSource
    public byte[] getRow(int y, byte[] row) {
        if (y < 0 || y >= getHeight()) {
            throw new IllegalArgumentException("Requested row is outside the image: ".concat(String.valueOf(y)));
        }
        int width = getWidth();
        if (row == null || row.length < width) {
            row = new byte[width];
        }
        int offset = ((this.top + y) * this.dataWidth) + this.left;
        System.arraycopy(this.yuvData, offset, row, 0, width);
        return row;
    }

    @Override // com.google.zxing.LuminanceSource
    public byte[] getMatrix() {
        int width = getWidth();
        int height = getHeight();
        if (width == this.dataWidth && height == this.dataHeight) {
            return this.yuvData;
        }
        int area = width * height;
        byte[] matrix = new byte[area];
        int inputOffset = (this.top * this.dataWidth) + this.left;
        if (width == this.dataWidth) {
            System.arraycopy(this.yuvData, inputOffset, matrix, 0, area);
            return matrix;
        }
        for (int y = 0; y < height; y++) {
            int outputOffset = y * width;
            System.arraycopy(this.yuvData, inputOffset, matrix, outputOffset, width);
            inputOffset += this.dataWidth;
        }
        return matrix;
    }

    @Override // com.google.zxing.LuminanceSource
    public boolean isCropSupported() {
        return true;
    }

    @Override // com.google.zxing.LuminanceSource
    public LuminanceSource crop(int left, int top, int width, int height) {
        return new PlanarYUVLuminanceSource(this.yuvData, this.dataWidth, this.dataHeight, this.left + left, this.top + top, width, height, false);
    }

    public int[] renderThumbnail() {
        int width = getWidth() / 2;
        int height = getHeight() / 2;
        int[] pixels = new int[width * height];
        byte[] yuv = this.yuvData;
        int inputOffset = (this.top * this.dataWidth) + this.left;
        for (int y = 0; y < height; y++) {
            int outputOffset = y * width;
            for (int x = 0; x < width; x++) {
                int grey = yuv[(x << 1) + inputOffset] & 255;
                pixels[outputOffset + x] = (65793 * grey) | ViewCompat.MEASURED_STATE_MASK;
            }
            int x2 = this.dataWidth;
            inputOffset += x2 << 1;
        }
        return pixels;
    }

    public int getThumbnailWidth() {
        return getWidth() / 2;
    }

    public int getThumbnailHeight() {
        return getHeight() / 2;
    }

    private void reverseHorizontal(int width, int height) {
        byte[] yuvData = this.yuvData;
        int y = 0;
        int rowStart = (this.top * this.dataWidth) + this.left;
        while (y < height) {
            int middle = (width / 2) + rowStart;
            int x1 = rowStart;
            int x2 = (rowStart + width) - 1;
            while (x1 < middle) {
                byte temp = yuvData[x1];
                yuvData[x1] = yuvData[x2];
                yuvData[x2] = temp;
                x1++;
                x2--;
            }
            y++;
            rowStart += this.dataWidth;
        }
    }
}
