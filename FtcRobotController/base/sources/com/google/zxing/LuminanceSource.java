package com.google.zxing;

import dk.sgjesse.r8api.DescriptorUtils;

/* JADX INFO: loaded from: classes8.dex */
public abstract class LuminanceSource {
    private final int height;
    private final int width;

    public abstract byte[] getMatrix();

    public abstract byte[] getRow(int i, byte[] bArr);

    protected LuminanceSource(int width, int height) {
        this.width = width;
        this.height = height;
    }

    public final int getWidth() {
        return this.width;
    }

    public final int getHeight() {
        return this.height;
    }

    public boolean isCropSupported() {
        return false;
    }

    public LuminanceSource crop(int left, int top, int width, int height) {
        throw new UnsupportedOperationException("This luminance source does not support cropping.");
    }

    public boolean isRotateSupported() {
        return false;
    }

    public LuminanceSource invert() {
        return new InvertedLuminanceSource(this);
    }

    public LuminanceSource rotateCounterClockwise() {
        throw new UnsupportedOperationException("This luminance source does not support rotation by 90 degrees.");
    }

    public LuminanceSource rotateCounterClockwise45() {
        throw new UnsupportedOperationException("This luminance source does not support rotation by 45 degrees.");
    }

    public final String toString() {
        char c;
        byte[] row = new byte[this.width];
        StringBuilder result = new StringBuilder(this.height * (this.width + 1));
        for (int y = 0; y < this.height; y++) {
            row = getRow(y, row);
            for (int x = 0; x < this.width; x++) {
                int luminance = row[x] & 255;
                if (luminance >= 64) {
                    if (luminance >= 128) {
                        if (luminance < 192) {
                            c = DescriptorUtils.JAVA_PACKAGE_SEPARATOR;
                        } else {
                            c = ' ';
                        }
                    } else {
                        c = '+';
                    }
                } else {
                    c = '#';
                }
                result.append(c);
            }
            result.append('\n');
        }
        return result.toString();
    }
}
