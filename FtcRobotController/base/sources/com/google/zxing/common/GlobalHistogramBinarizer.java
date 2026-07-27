package com.google.zxing.common;

import com.google.zxing.Binarizer;
import com.google.zxing.LuminanceSource;
import com.google.zxing.NotFoundException;

/* JADX INFO: loaded from: classes8.dex */
public class GlobalHistogramBinarizer extends Binarizer {
    private static final byte[] EMPTY = new byte[0];
    private static final int LUMINANCE_BITS = 5;
    private static final int LUMINANCE_BUCKETS = 32;
    private static final int LUMINANCE_SHIFT = 3;
    private final int[] buckets;
    private byte[] luminances;

    public GlobalHistogramBinarizer(LuminanceSource source) {
        super(source);
        this.luminances = EMPTY;
        this.buckets = new int[32];
    }

    @Override // com.google.zxing.Binarizer
    public BitArray getBlackRow(int y, BitArray row) throws NotFoundException {
        LuminanceSource source = getLuminanceSource();
        int width = source.getWidth();
        if (row == null || row.getSize() < width) {
            row = new BitArray(width);
        } else {
            row.clear();
        }
        initArrays(width);
        byte[] localLuminances = source.getRow(y, this.luminances);
        int[] localBuckets = this.buckets;
        for (int x = 0; x < width; x++) {
            int i = (localLuminances[x] & 255) >> 3;
            localBuckets[i] = localBuckets[i] + 1;
        }
        int blackPoint = estimateBlackPoint(localBuckets);
        if (width < 3) {
            for (int x2 = 0; x2 < width; x2++) {
                if ((localLuminances[x2] & 255) < blackPoint) {
                    row.set(x2);
                }
            }
        } else {
            int left = localLuminances[0] & 255;
            int center = localLuminances[1] & 255;
            for (int x3 = 1; x3 < width - 1; x3++) {
                int right = localLuminances[x3 + 1] & 255;
                if ((((center << 2) - left) - right) / 2 < blackPoint) {
                    row.set(x3);
                }
                left = center;
                center = right;
            }
        }
        return row;
    }

    @Override // com.google.zxing.Binarizer
    public BitMatrix getBlackMatrix() throws NotFoundException {
        LuminanceSource luminanceSource = getLuminanceSource();
        int width = luminanceSource.getWidth();
        int height = luminanceSource.getHeight();
        BitMatrix bitMatrix = new BitMatrix(width, height);
        initArrays(width);
        int[] iArr = this.buckets;
        for (int i = 1; i < 5; i++) {
            byte[] row = luminanceSource.getRow((height * i) / 5, this.luminances);
            int i2 = (width << 2) / 5;
            for (int i3 = width / 5; i3 < i2; i3++) {
                int i4 = (row[i3] & 255) >> 3;
                iArr[i4] = iArr[i4] + 1;
            }
        }
        int iEstimateBlackPoint = estimateBlackPoint(iArr);
        byte[] matrix = luminanceSource.getMatrix();
        for (int i5 = 0; i5 < height; i5++) {
            int i6 = i5 * width;
            for (int i7 = 0; i7 < width; i7++) {
                if ((matrix[i6 + i7] & 255) < iEstimateBlackPoint) {
                    bitMatrix.set(i7, i5);
                }
            }
        }
        return bitMatrix;
    }

    @Override // com.google.zxing.Binarizer
    public Binarizer createBinarizer(LuminanceSource source) {
        return new GlobalHistogramBinarizer(source);
    }

    private void initArrays(int luminanceSize) {
        if (this.luminances.length < luminanceSize) {
            this.luminances = new byte[luminanceSize];
        }
        for (int x = 0; x < 32; x++) {
            this.buckets[x] = 0;
        }
    }

    private static int estimateBlackPoint(int[] iArr) throws NotFoundException {
        int length = iArr.length;
        int i = 0;
        int i2 = 0;
        int i3 = 0;
        for (int i4 = 0; i4 < length; i4++) {
            if (iArr[i4] > i) {
                i = iArr[i4];
                i3 = i4;
            }
            if (iArr[i4] > i2) {
                i2 = iArr[i4];
            }
        }
        int i5 = 0;
        int i6 = 0;
        for (int i7 = 0; i7 < length; i7++) {
            int i8 = i7 - i3;
            int i9 = iArr[i7] * i8 * i8;
            if (i9 > i6) {
                i5 = i7;
                i6 = i9;
            }
        }
        if (i3 <= i5) {
            int i10 = i3;
            i3 = i5;
            i5 = i10;
        }
        if (i3 - i5 <= length / 16) {
            throw NotFoundException.getNotFoundInstance();
        }
        int i11 = i3 - 1;
        int i12 = -1;
        int i13 = i11;
        while (i11 > i5) {
            int i14 = i11 - i5;
            int i15 = i14 * i14 * (i3 - i11) * (i2 - iArr[i11]);
            if (i15 > i12) {
                i13 = i11;
                i12 = i15;
            }
            i11--;
        }
        return i13 << 3;
    }
}
