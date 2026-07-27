package com.google.zxing;

/* JADX INFO: loaded from: classes8.dex */
public final class InvertedLuminanceSource extends LuminanceSource {
    private final LuminanceSource delegate;

    public InvertedLuminanceSource(LuminanceSource delegate) {
        super(delegate.getWidth(), delegate.getHeight());
        this.delegate = delegate;
    }

    @Override // com.google.zxing.LuminanceSource
    public byte[] getRow(int y, byte[] row) {
        byte[] row2 = this.delegate.getRow(y, row);
        int width = getWidth();
        for (int i = 0; i < width; i++) {
            row2[i] = (byte) (255 - (row2[i] & 255));
        }
        return row2;
    }

    @Override // com.google.zxing.LuminanceSource
    public byte[] getMatrix() {
        byte[] matrix = this.delegate.getMatrix();
        int length = getWidth() * getHeight();
        byte[] invertedMatrix = new byte[length];
        for (int i = 0; i < length; i++) {
            invertedMatrix[i] = (byte) (255 - (matrix[i] & 255));
        }
        return invertedMatrix;
    }

    @Override // com.google.zxing.LuminanceSource
    public boolean isCropSupported() {
        return this.delegate.isCropSupported();
    }

    @Override // com.google.zxing.LuminanceSource
    public LuminanceSource crop(int left, int top, int width, int height) {
        return new InvertedLuminanceSource(this.delegate.crop(left, top, width, height));
    }

    @Override // com.google.zxing.LuminanceSource
    public boolean isRotateSupported() {
        return this.delegate.isRotateSupported();
    }

    @Override // com.google.zxing.LuminanceSource
    public LuminanceSource invert() {
        return this.delegate;
    }

    @Override // com.google.zxing.LuminanceSource
    public LuminanceSource rotateCounterClockwise() {
        return new InvertedLuminanceSource(this.delegate.rotateCounterClockwise());
    }

    @Override // com.google.zxing.LuminanceSource
    public LuminanceSource rotateCounterClockwise45() {
        return new InvertedLuminanceSource(this.delegate.rotateCounterClockwise45());
    }
}
