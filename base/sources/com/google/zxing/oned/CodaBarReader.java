package com.google.zxing.oned;

import com.google.zxing.BarcodeFormat;
import com.google.zxing.DecodeHintType;
import com.google.zxing.NotFoundException;
import com.google.zxing.Result;
import com.google.zxing.ResultPoint;
import com.google.zxing.common.BitArray;
import java.util.Arrays;
import java.util.Map;

/* JADX INFO: loaded from: classes8.dex */
public final class CodaBarReader extends OneDReader {
    private static final float MAX_ACCEPTABLE = 2.0f;
    private static final int MIN_CHARACTER_LENGTH = 3;
    private static final float PADDING = 1.5f;
    private static final String ALPHABET_STRING = "0123456789-$:/.+ABCD";
    static final char[] ALPHABET = ALPHABET_STRING.toCharArray();
    static final int[] CHARACTER_ENCODINGS = {3, 6, 9, 96, 18, 66, 33, 36, 48, 72, 12, 24, 69, 81, 84, 21, 26, 41, 11, 14};
    private static final char[] STARTEND_ENCODING = {'A', 'B', 'C', 'D'};
    private final StringBuilder decodeRowResult = new StringBuilder(20);
    private int[] counters = new int[80];
    private int counterLength = 0;

    @Override // com.google.zxing.oned.OneDReader
    public Result decodeRow(int i, BitArray bitArray, Map<DecodeHintType, ?> map) throws NotFoundException {
        Arrays.fill(this.counters, 0);
        setCounters(bitArray);
        int iFindStartPattern = findStartPattern();
        this.decodeRowResult.setLength(0);
        int i2 = iFindStartPattern;
        do {
            int narrowWidePattern = toNarrowWidePattern(i2);
            if (narrowWidePattern == -1) {
                throw NotFoundException.getNotFoundInstance();
            }
            this.decodeRowResult.append((char) narrowWidePattern);
            i2 += 8;
            if (this.decodeRowResult.length() > 1 && arrayContains(STARTEND_ENCODING, ALPHABET[narrowWidePattern])) {
                break;
            }
        } while (i2 < this.counterLength);
        int i3 = i2 - 1;
        int i4 = this.counters[i3];
        int i5 = 0;
        for (int i6 = -8; i6 < -1; i6++) {
            i5 += this.counters[i2 + i6];
        }
        if (i2 < this.counterLength && i4 < i5 / 2) {
            throw NotFoundException.getNotFoundInstance();
        }
        validatePattern(iFindStartPattern);
        for (int i7 = 0; i7 < this.decodeRowResult.length(); i7++) {
            this.decodeRowResult.setCharAt(i7, ALPHABET[this.decodeRowResult.charAt(i7)]);
        }
        if (!arrayContains(STARTEND_ENCODING, this.decodeRowResult.charAt(0))) {
            throw NotFoundException.getNotFoundInstance();
        }
        if (!arrayContains(STARTEND_ENCODING, this.decodeRowResult.charAt(this.decodeRowResult.length() - 1))) {
            throw NotFoundException.getNotFoundInstance();
        }
        if (this.decodeRowResult.length() <= 3) {
            throw NotFoundException.getNotFoundInstance();
        }
        if (map == null || !map.containsKey(DecodeHintType.RETURN_CODABAR_START_END)) {
            this.decodeRowResult.deleteCharAt(this.decodeRowResult.length() - 1);
            this.decodeRowResult.deleteCharAt(0);
        }
        int i8 = 0;
        for (int i9 = 0; i9 < iFindStartPattern; i9++) {
            i8 += this.counters[i9];
        }
        float f = i8;
        while (iFindStartPattern < i3) {
            i8 += this.counters[iFindStartPattern];
            iFindStartPattern++;
        }
        float f2 = i;
        return new Result(this.decodeRowResult.toString(), null, new ResultPoint[]{new ResultPoint(f, f2), new ResultPoint(i8, f2)}, BarcodeFormat.CODABAR);
    }

    /* JADX WARN: Code restructure failed: missing block: B:21:0x00bc, code lost:
    
        throw com.google.zxing.NotFoundException.getNotFoundInstance();
     */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    private void validatePattern(int r14) throws com.google.zxing.NotFoundException {
        /*
            r13 = this;
            r0 = 0
            int[] r1 = new int[]{r0, r0, r0, r0}
            int[] r0 = new int[]{r0, r0, r0, r0}
            java.lang.StringBuilder r2 = r13.decodeRowResult
            int r2 = r2.length()
            int r2 = r2 + (-1)
            r3 = r14
            r4 = 0
        L13:
            int[] r5 = com.google.zxing.oned.CodaBarReader.CHARACTER_ENCODINGS
            java.lang.StringBuilder r6 = r13.decodeRowResult
            char r6 = r6.charAt(r4)
            r5 = r5[r6]
            r6 = 6
        L1e:
            if (r6 < 0) goto L3d
            r7 = r6 & 1
            r8 = r5 & 1
            int r8 = r8 << 1
            int r7 = r7 + r8
            r8 = r1[r7]
            int[] r9 = r13.counters
            int r10 = r3 + r6
            r9 = r9[r10]
            int r8 = r8 + r9
            r1[r7] = r8
            r8 = r0[r7]
            int r8 = r8 + 1
            r0[r7] = r8
            int r5 = r5 >> 1
            int r6 = r6 + (-1)
            goto L1e
        L3d:
            if (r4 >= r2) goto L45
        L40:
            int r3 = r3 + 8
            int r4 = r4 + 1
            goto L13
        L45:
            r4 = 4
            float[] r5 = new float[r4]
            float[] r4 = new float[r4]
            r6 = 0
        L4b:
            r7 = 2
            if (r6 >= r7) goto L87
            r7 = 0
            r4[r6] = r7
            int r7 = r6 + 2
            r8 = r1[r6]
            float r8 = (float) r8
            r9 = r0[r6]
            float r9 = (float) r9
            float r8 = r8 / r9
            int r9 = r6 + 2
            r9 = r1[r9]
            float r9 = (float) r9
            int r10 = r6 + 2
            r10 = r0[r10]
            float r10 = (float) r10
            float r9 = r9 / r10
            float r8 = r8 + r9
            r9 = 1073741824(0x40000000, float:2.0)
            float r8 = r8 / r9
            r4[r7] = r8
            int r7 = r6 + 2
            r7 = r4[r7]
            r5[r6] = r7
            int r7 = r6 + 2
            int r8 = r6 + 2
            r8 = r1[r8]
            float r8 = (float) r8
            float r8 = r8 * r9
            r9 = 1069547520(0x3fc00000, float:1.5)
            float r8 = r8 + r9
            int r9 = r6 + 2
            r9 = r0[r9]
            float r9 = (float) r9
            float r8 = r8 / r9
            r5[r7] = r8
            int r6 = r6 + 1
            goto L4b
        L87:
            r3 = r14
            r6 = 0
            r7 = 0
        L8a:
            int[] r8 = com.google.zxing.oned.CodaBarReader.CHARACTER_ENCODINGS
            java.lang.StringBuilder r9 = r13.decodeRowResult
            char r9 = r9.charAt(r6)
            r8 = r8[r9]
            r9 = 6
        L95:
            if (r9 < 0) goto Lbd
            r10 = r9 & 1
            r11 = r8 & 1
            int r11 = r11 << 1
            int r10 = r10 + r11
            int[] r11 = r13.counters
            int r12 = r3 + r9
            r11 = r11[r12]
            r7 = r11
            float r11 = (float) r11
            r12 = r4[r10]
            int r11 = (r11 > r12 ? 1 : (r11 == r12 ? 0 : -1))
            if (r11 < 0) goto Lb8
            float r11 = (float) r7
            r12 = r5[r10]
            int r11 = (r11 > r12 ? 1 : (r11 == r12 ? 0 : -1))
            if (r11 > 0) goto Lb8
            int r8 = r8 >> 1
            int r9 = r9 + (-1)
            goto L95
        Lb8:
            com.google.zxing.NotFoundException r11 = com.google.zxing.NotFoundException.getNotFoundInstance()
            throw r11
        Lbd:
            if (r6 >= r2) goto Lc5
        Lc0:
            int r3 = r3 + 8
            int r6 = r6 + 1
            goto L8a
        Lc5:
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: com.google.zxing.oned.CodaBarReader.validatePattern(int):void");
    }

    private void setCounters(BitArray row) throws NotFoundException {
        this.counterLength = 0;
        int i = row.getNextUnset(0);
        int end = row.getSize();
        if (i >= end) {
            throw NotFoundException.getNotFoundInstance();
        }
        boolean isWhite = true;
        int count = 0;
        while (i < end) {
            if (row.get(i) != isWhite) {
                count++;
            } else {
                counterAppend(count);
                count = 1;
                isWhite = !isWhite;
            }
            i++;
        }
        counterAppend(count);
    }

    private void counterAppend(int e) {
        this.counters[this.counterLength] = e;
        this.counterLength++;
        if (this.counterLength >= this.counters.length) {
            int[] temp = new int[this.counterLength << 1];
            System.arraycopy(this.counters, 0, temp, 0, this.counterLength);
            this.counters = temp;
        }
    }

    private int findStartPattern() throws NotFoundException {
        for (int i = 1; i < this.counterLength; i += 2) {
            int charOffset = toNarrowWidePattern(i);
            if (charOffset != -1 && arrayContains(STARTEND_ENCODING, ALPHABET[charOffset])) {
                int patternSize = 0;
                for (int j = i; j < i + 7; j++) {
                    patternSize += this.counters[j];
                }
                if (i == 1 || this.counters[i - 1] >= patternSize / 2) {
                    return i;
                }
            }
        }
        throw NotFoundException.getNotFoundInstance();
    }

    static boolean arrayContains(char[] array, char key) {
        if (array != null) {
            for (char c : array) {
                if (c == key) {
                    return true;
                }
            }
        }
        return false;
    }

    private int toNarrowWidePattern(int i) {
        int i2 = i + 7;
        if (i2 >= this.counterLength) {
            return -1;
        }
        int[] iArr = this.counters;
        int i3 = Integer.MAX_VALUE;
        int i4 = Integer.MAX_VALUE;
        int i5 = 0;
        for (int i6 = i; i6 < i2; i6 += 2) {
            int i7 = iArr[i6];
            if (i7 < i4) {
                i4 = i7;
            }
            if (i7 > i5) {
                i5 = i7;
            }
        }
        int i8 = (i4 + i5) / 2;
        int i9 = 0;
        for (int i10 = i + 1; i10 < i2; i10 += 2) {
            int i11 = iArr[i10];
            if (i11 < i3) {
                i3 = i11;
            }
            if (i11 > i9) {
                i9 = i11;
            }
        }
        int i12 = (i3 + i9) / 2;
        int i13 = 128;
        int i14 = 0;
        for (int i15 = 0; i15 < 7; i15++) {
            i13 >>= 1;
            if (iArr[i + i15] > ((i15 & 1) == 0 ? i8 : i12)) {
                i14 |= i13;
            }
        }
        for (int i16 = 0; i16 < CHARACTER_ENCODINGS.length; i16++) {
            if (CHARACTER_ENCODINGS[i16] == i14) {
                return i16;
            }
        }
        return -1;
    }
}
