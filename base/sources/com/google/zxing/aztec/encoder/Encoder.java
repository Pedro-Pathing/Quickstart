package com.google.zxing.aztec.encoder;

import com.google.zxing.common.BitArray;
import com.google.zxing.common.BitMatrix;
import com.google.zxing.common.reedsolomon.GenericGF;
import com.google.zxing.common.reedsolomon.ReedSolomonEncoder;

/* JADX INFO: loaded from: classes8.dex */
public final class Encoder {
    public static final int DEFAULT_AZTEC_LAYERS = 0;
    public static final int DEFAULT_EC_PERCENT = 33;
    private static final int MAX_NB_BITS = 32;
    private static final int MAX_NB_BITS_COMPACT = 4;
    private static final int[] WORD_SIZE = {4, 6, 6, 8, 8, 8, 8, 8, 8, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12};

    private Encoder() {
    }

    public static AztecCode encode(byte[] data) {
        return encode(data, 33, 0);
    }

    /* JADX WARN: Multi-variable type inference failed */
    public static AztecCode encode(byte[] bArr, int i, int i2) {
        BitArray bitArrayStuffBits;
        int i3;
        boolean z;
        int iAbs;
        int i4;
        int i5;
        BitArray bitArrayEncode = new HighLevelEncoder(bArr).encode();
        int size = ((bitArrayEncode.getSize() * i) / 100) + 11;
        int size2 = bitArrayEncode.getSize() + size;
        int i6 = 1;
        if (i2 != 0) {
            boolean z2 = i2 < 0;
            iAbs = Math.abs(i2);
            if (iAbs > (z2 ? 4 : 32)) {
                throw new IllegalArgumentException(String.format("Illegal value %s for layers", Integer.valueOf(i2)));
            }
            i4 = totalBitsInLayer(iAbs, z2);
            i3 = WORD_SIZE[iAbs];
            int i7 = i4 - (i4 % i3);
            bitArrayStuffBits = stuffBits(bitArrayEncode, i3);
            z = z2;
            if (bitArrayStuffBits.getSize() + size > i7) {
                throw new IllegalArgumentException("Data to large for user specified layer");
            }
            if (z2) {
                z = z2;
                if (bitArrayStuffBits.getSize() > (i3 << 6)) {
                    throw new IllegalArgumentException("Data to large for user specified layer");
                }
            }
        } else {
            BitArray bitArrayStuffBits2 = null;
            int i8 = 0;
            int i9 = 0;
            while (i8 <= 32) {
                boolean z3 = i8 <= 3 ? i6 : 0;
                int i10 = z3 != 0 ? i8 + 1 : i8;
                int i11 = totalBitsInLayer(i10, z3);
                if (size2 <= i11) {
                    if (bitArrayStuffBits2 == null || i9 != WORD_SIZE[i10]) {
                        int i12 = WORD_SIZE[i10];
                        i9 = i12;
                        bitArrayStuffBits2 = stuffBits(bitArrayEncode, i12);
                    }
                    int i13 = i11 - (i11 % i9);
                    if ((z3 == 0 || bitArrayStuffBits2.getSize() <= (i9 << 6)) && bitArrayStuffBits2.getSize() + size <= i13) {
                        bitArrayStuffBits = bitArrayStuffBits2;
                        i3 = i9;
                        z = z3;
                        iAbs = i10;
                        i4 = i11;
                    }
                }
                i8++;
                i6 = i6;
            }
            throw new IllegalArgumentException("Data too large for an Aztec code");
        }
        BitArray bitArrayGenerateCheckWords = generateCheckWords(bitArrayStuffBits, i4, i3);
        int size3 = bitArrayStuffBits.getSize() / i3;
        BitArray bitArrayGenerateModeMessage = generateModeMessage(z, iAbs, size3);
        int i14 = (z ? 11 : 14) + (iAbs << 2);
        int[] iArr = new int[i14];
        int i15 = 2;
        if (!z) {
            int i16 = i14 / 2;
            i5 = i14 + 1 + (((i16 - 1) / 15) * 2);
            int i17 = i5 / 2;
            for (int i18 = 0; i18 < i16; i18++) {
                iArr[(i16 - i18) - i6] = (i17 - r14) - 1;
                iArr[i16 + i18] = (i18 / 15) + i18 + i17 + i6;
            }
        } else {
            for (int i19 = 0; i19 < i14; i19++) {
                iArr[i19] = i19;
            }
            i5 = i14;
        }
        BitMatrix bitMatrix = new BitMatrix(i5);
        int i20 = 0;
        int i21 = 0;
        while (i20 < iAbs) {
            int i22 = ((iAbs - i20) << i15) + (z ? 9 : 12);
            int i23 = 0;
            while (i23 < i22) {
                int i24 = i23 << 1;
                int i25 = 0;
                while (i25 < i15) {
                    if (bitArrayGenerateCheckWords.get(i21 + i24 + i25)) {
                        int i26 = i20 << 1;
                        bitMatrix.set(iArr[i26 + i25], iArr[i26 + i23]);
                    }
                    if (bitArrayGenerateCheckWords.get((i22 << 1) + i21 + i24 + i25)) {
                        int i27 = i20 << 1;
                        bitMatrix.set(iArr[i27 + i23], iArr[((i14 - 1) - i27) - i25]);
                    }
                    if (bitArrayGenerateCheckWords.get((i22 << 2) + i21 + i24 + i25)) {
                        int i28 = (i14 - 1) - (i20 << 1);
                        bitMatrix.set(iArr[i28 - i25], iArr[i28 - i23]);
                    }
                    if (bitArrayGenerateCheckWords.get((i22 * 6) + i21 + i24 + i25)) {
                        int i29 = i20 << 1;
                        bitMatrix.set(iArr[((i14 - 1) - i29) - i23], iArr[i29 + i25]);
                    }
                    i25++;
                    i15 = 2;
                }
                i23++;
                i15 = 2;
            }
            i21 += i22 << 3;
            i20++;
            i15 = 2;
        }
        drawModeMessage(bitMatrix, z, i5, bitArrayGenerateModeMessage);
        if (z) {
            drawBullsEye(bitMatrix, i5 / 2, 5);
        } else {
            int i30 = i5 / 2;
            drawBullsEye(bitMatrix, i30, 7);
            int i31 = 0;
            int i32 = 0;
            while (i32 < (i14 / 2) - 1) {
                for (int i33 = i30 & 1; i33 < i5; i33 += 2) {
                    int i34 = i30 - i31;
                    bitMatrix.set(i34, i33);
                    int i35 = i30 + i31;
                    bitMatrix.set(i35, i33);
                    bitMatrix.set(i33, i34);
                    bitMatrix.set(i33, i35);
                }
                i32 += 15;
                i31 += 16;
            }
        }
        AztecCode aztecCode = new AztecCode();
        aztecCode.setCompact(z);
        aztecCode.setSize(i5);
        aztecCode.setLayers(iAbs);
        aztecCode.setCodeWords(size3);
        aztecCode.setMatrix(bitMatrix);
        return aztecCode;
    }

    private static void drawBullsEye(BitMatrix matrix, int center, int size) {
        for (int i = 0; i < size; i += 2) {
            for (int j = center - i; j <= center + i; j++) {
                matrix.set(j, center - i);
                matrix.set(j, center + i);
                matrix.set(center - i, j);
                matrix.set(center + i, j);
            }
        }
        int i2 = center - size;
        matrix.set(i2, center - size);
        matrix.set((center - size) + 1, center - size);
        matrix.set(center - size, (center - size) + 1);
        matrix.set(center + size, center - size);
        matrix.set(center + size, (center - size) + 1);
        matrix.set(center + size, (center + size) - 1);
    }

    static BitArray generateModeMessage(boolean compact, int layers, int messageSizeInWords) {
        BitArray modeMessage = new BitArray();
        if (compact) {
            modeMessage.appendBits(layers - 1, 2);
            modeMessage.appendBits(messageSizeInWords - 1, 6);
            return generateCheckWords(modeMessage, 28, 4);
        }
        modeMessage.appendBits(layers - 1, 5);
        modeMessage.appendBits(messageSizeInWords - 1, 11);
        return generateCheckWords(modeMessage, 40, 4);
    }

    private static void drawModeMessage(BitMatrix matrix, boolean compact, int matrixSize, BitArray modeMessage) {
        int center = matrixSize / 2;
        if (compact) {
            for (int i = 0; i < 7; i++) {
                int offset = (center - 3) + i;
                if (modeMessage.get(i)) {
                    matrix.set(offset, center - 5);
                }
                if (modeMessage.get(i + 7)) {
                    matrix.set(center + 5, offset);
                }
                if (modeMessage.get(20 - i)) {
                    matrix.set(offset, center + 5);
                }
                if (modeMessage.get(27 - i)) {
                    matrix.set(center - 5, offset);
                }
            }
            return;
        }
        for (int i2 = 0; i2 < 10; i2++) {
            int offset2 = (center - 5) + i2 + (i2 / 5);
            if (modeMessage.get(i2)) {
                matrix.set(offset2, center - 7);
            }
            if (modeMessage.get(i2 + 10)) {
                matrix.set(center + 7, offset2);
            }
            if (modeMessage.get(29 - i2)) {
                matrix.set(offset2, center + 7);
            }
            if (modeMessage.get(39 - i2)) {
                matrix.set(center - 7, offset2);
            }
        }
    }

    private static BitArray generateCheckWords(BitArray bitArray, int totalBits, int wordSize) {
        int messageSizeInWords = bitArray.getSize() / wordSize;
        ReedSolomonEncoder rs = new ReedSolomonEncoder(getGF(wordSize));
        int totalWords = totalBits / wordSize;
        int[] messageWords = bitsToWords(bitArray, wordSize, totalWords);
        rs.encode(messageWords, totalWords - messageSizeInWords);
        int startPad = totalBits % wordSize;
        BitArray messageBits = new BitArray();
        messageBits.appendBits(0, startPad);
        for (int messageWord : messageWords) {
            messageBits.appendBits(messageWord, wordSize);
        }
        return messageBits;
    }

    private static int[] bitsToWords(BitArray stuffedBits, int wordSize, int totalWords) {
        int[] message = new int[totalWords];
        int n = stuffedBits.getSize() / wordSize;
        for (int i = 0; i < n; i++) {
            int value = 0;
            for (int j = 0; j < wordSize; j++) {
                value |= stuffedBits.get((i * wordSize) + j) ? 1 << ((wordSize - j) - 1) : 0;
            }
            message[i] = value;
        }
        return message;
    }

    private static GenericGF getGF(int wordSize) {
        switch (wordSize) {
            case 4:
                return GenericGF.AZTEC_PARAM;
            case 5:
            case 7:
            case 9:
            case 11:
            default:
                throw new IllegalArgumentException("Unsupported word size ".concat(String.valueOf(wordSize)));
            case 6:
                return GenericGF.AZTEC_DATA_6;
            case 8:
                return GenericGF.AZTEC_DATA_8;
            case 10:
                return GenericGF.AZTEC_DATA_10;
            case 12:
                return GenericGF.AZTEC_DATA_12;
        }
    }

    static BitArray stuffBits(BitArray bits, int wordSize) {
        BitArray out = new BitArray();
        int n = bits.getSize();
        int mask = (1 << wordSize) - 2;
        int i = 0;
        while (i < n) {
            int word = 0;
            for (int j = 0; j < wordSize; j++) {
                if (i + j >= n || bits.get(i + j)) {
                    word |= 1 << ((wordSize - 1) - j);
                }
            }
            int j2 = word & mask;
            if (j2 == mask) {
                out.appendBits(word & mask, wordSize);
                i--;
            } else if ((word & mask) == 0) {
                out.appendBits(word | 1, wordSize);
                i--;
            } else {
                out.appendBits(word, wordSize);
            }
            i += wordSize;
        }
        return out;
    }

    private static int totalBitsInLayer(int layers, boolean compact) {
        return ((compact ? 88 : 112) + (layers << 4)) * layers;
    }
}
