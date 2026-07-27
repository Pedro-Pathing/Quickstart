package com.google.zxing.common;

import com.google.zxing.NotFoundException;

/* JADX INFO: loaded from: classes8.dex */
public final class DefaultGridSampler extends GridSampler {
    @Override // com.google.zxing.common.GridSampler
    public BitMatrix sampleGrid(BitMatrix image, int dimensionX, int dimensionY, float p1ToX, float p1ToY, float p2ToX, float p2ToY, float p3ToX, float p3ToY, float p4ToX, float p4ToY, float p1FromX, float p1FromY, float p2FromX, float p2FromY, float p3FromX, float p3FromY, float p4FromX, float p4FromY) throws NotFoundException {
        PerspectiveTransform transform = PerspectiveTransform.quadrilateralToQuadrilateral(p1ToX, p1ToY, p2ToX, p2ToY, p3ToX, p3ToY, p4ToX, p4ToY, p1FromX, p1FromY, p2FromX, p2FromY, p3FromX, p3FromY, p4FromX, p4FromY);
        return sampleGrid(image, dimensionX, dimensionY, transform);
    }

    @Override // com.google.zxing.common.GridSampler
    public BitMatrix sampleGrid(BitMatrix bitMatrix, int i, int i2, PerspectiveTransform perspectiveTransform) throws NotFoundException {
        if (i <= 0 || i2 <= 0) {
            throw NotFoundException.getNotFoundInstance();
        }
        BitMatrix bitMatrix2 = new BitMatrix(i, i2);
        int i3 = i * 2;
        float[] fArr = new float[i3];
        for (int i4 = 0; i4 < i2; i4++) {
            float f = i4 + 0.5f;
            for (int i5 = 0; i5 < i3; i5 += 2) {
                fArr[i5] = (i5 / 2) + 0.5f;
                fArr[i5 + 1] = f;
            }
            perspectiveTransform.transformPoints(fArr);
            checkAndNudgePoints(bitMatrix, fArr);
            for (int i6 = 0; i6 < i3; i6 += 2) {
                try {
                    if (bitMatrix.get((int) fArr[i6], (int) fArr[i6 + 1])) {
                        bitMatrix2.set(i6 / 2, i4);
                    }
                } catch (ArrayIndexOutOfBoundsException e) {
                    throw NotFoundException.getNotFoundInstance();
                }
            }
        }
        return bitMatrix2;
    }
}
