package com.google.zxing.oned;

import com.google.zxing.BarcodeFormat;
import com.google.zxing.NotFoundException;
import com.google.zxing.common.BitArray;

/* JADX INFO: loaded from: classes8.dex */
public final class EAN13Reader extends UPCEANReader {
    static final int[] FIRST_DIGIT_ENCODINGS = {0, 11, 13, 14, 19, 25, 28, 21, 22, 26};
    private final int[] decodeMiddleCounters = new int[4];

    @Override // com.google.zxing.oned.UPCEANReader
    protected int decodeMiddle(BitArray bitArray, int[] iArr, StringBuilder sb) throws NotFoundException {
        int[] iArr2 = this.decodeMiddleCounters;
        iArr2[0] = 0;
        iArr2[1] = 0;
        iArr2[2] = 0;
        iArr2[3] = 0;
        int size = bitArray.getSize();
        int i = iArr[1];
        int i2 = 0;
        for (int i3 = 0; i3 < 6 && i < size; i3++) {
            int iDecodeDigit = decodeDigit(bitArray, iArr2, i, L_AND_G_PATTERNS);
            sb.append((char) ((iDecodeDigit % 10) + 48));
            for (int i4 : iArr2) {
                i += i4;
            }
            if (iDecodeDigit >= 10) {
                i2 |= 1 << (5 - i3);
            }
        }
        determineFirstDigit(sb, i2);
        int i5 = findGuardPattern(bitArray, i, true, MIDDLE_PATTERN)[1];
        for (int i6 = 0; i6 < 6 && i5 < size; i6++) {
            sb.append((char) (decodeDigit(bitArray, iArr2, i5, L_PATTERNS) + 48));
            for (int i7 : iArr2) {
                i5 += i7;
            }
        }
        return i5;
    }

    @Override // com.google.zxing.oned.UPCEANReader
    BarcodeFormat getBarcodeFormat() {
        return BarcodeFormat.EAN_13;
    }

    private static void determineFirstDigit(StringBuilder resultString, int lgPatternFound) throws NotFoundException {
        for (int d = 0; d < 10; d++) {
            if (lgPatternFound == FIRST_DIGIT_ENCODINGS[d]) {
                resultString.insert(0, (char) (d + 48));
                return;
            }
        }
        throw NotFoundException.getNotFoundInstance();
    }
}
