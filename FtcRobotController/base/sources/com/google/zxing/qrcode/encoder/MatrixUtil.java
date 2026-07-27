package com.google.zxing.qrcode.encoder;

import com.android.tools.r8.DataResource;
import com.google.zxing.WriterException;
import com.google.zxing.common.BitArray;
import com.google.zxing.qrcode.decoder.ErrorCorrectionLevel;
import com.google.zxing.qrcode.decoder.Version;
import com.sun.tools.javac.jvm.ByteCodes;

/* JADX INFO: loaded from: classes8.dex */
final class MatrixUtil {
    private static final int TYPE_INFO_MASK_PATTERN = 21522;
    private static final int TYPE_INFO_POLY = 1335;
    private static final int VERSION_INFO_POLY = 7973;
    private static final int[][] POSITION_DETECTION_PATTERN = {new int[]{1, 1, 1, 1, 1, 1, 1}, new int[]{1, 0, 0, 0, 0, 0, 1}, new int[]{1, 0, 1, 1, 1, 0, 1}, new int[]{1, 0, 1, 1, 1, 0, 1}, new int[]{1, 0, 1, 1, 1, 0, 1}, new int[]{1, 0, 0, 0, 0, 0, 1}, new int[]{1, 1, 1, 1, 1, 1, 1}};
    private static final int[][] POSITION_ADJUSTMENT_PATTERN = {new int[]{1, 1, 1, 1, 1}, new int[]{1, 0, 0, 0, 1}, new int[]{1, 0, 1, 0, 1}, new int[]{1, 0, 0, 0, 1}, new int[]{1, 1, 1, 1, 1}};
    private static final int[][] POSITION_ADJUSTMENT_PATTERN_COORDINATE_TABLE = {new int[]{-1, -1, -1, -1, -1, -1, -1}, new int[]{6, 18, -1, -1, -1, -1, -1}, new int[]{6, 22, -1, -1, -1, -1, -1}, new int[]{6, 26, -1, -1, -1, -1, -1}, new int[]{6, 30, -1, -1, -1, -1, -1}, new int[]{6, 34, -1, -1, -1, -1, -1}, new int[]{6, 22, 38, -1, -1, -1, -1}, new int[]{6, 24, 42, -1, -1, -1, -1}, new int[]{6, 26, 46, -1, -1, -1, -1}, new int[]{6, 28, 50, -1, -1, -1, -1}, new int[]{6, 30, 54, -1, -1, -1, -1}, new int[]{6, 32, 58, -1, -1, -1, -1}, new int[]{6, 34, 62, -1, -1, -1, -1}, new int[]{6, 26, 46, 66, -1, -1, -1}, new int[]{6, 26, 48, 70, -1, -1, -1}, new int[]{6, 26, 50, 74, -1, -1, -1}, new int[]{6, 30, 54, 78, -1, -1, -1}, new int[]{6, 30, 56, 82, -1, -1, -1}, new int[]{6, 30, 58, 86, -1, -1, -1}, new int[]{6, 34, 62, 90, -1, -1, -1}, new int[]{6, 28, 50, 72, 94, -1, -1}, new int[]{6, 26, 50, 74, 98, -1, -1}, new int[]{6, 30, 54, 78, 102, -1, -1}, new int[]{6, 28, 54, 80, 106, -1, -1}, new int[]{6, 32, 58, 84, 110, -1, -1}, new int[]{6, 30, 58, 86, ByteCodes.fmod, -1, -1}, new int[]{6, 34, 62, 90, 118, -1, -1}, new int[]{6, 26, 50, 74, 98, 122, -1}, new int[]{6, 30, 54, 78, 102, 126, -1}, new int[]{6, 26, 52, 78, 104, 130, -1}, new int[]{6, 30, 56, 82, 108, 134, -1}, new int[]{6, 34, 60, 86, 112, 138, -1}, new int[]{6, 30, 58, 86, ByteCodes.fmod, 142, -1}, new int[]{6, 34, 62, 90, 118, 146, -1}, new int[]{6, 30, 54, 78, 102, 126, 150}, new int[]{6, 24, 50, 76, 102, 128, 154}, new int[]{6, 28, 54, 80, 106, 132, ByteCodes.ifle}, new int[]{6, 32, 58, 84, 110, 136, ByteCodes.if_icmpge}, new int[]{6, 26, 54, 82, 110, 138, ByteCodes.if_acmpne}, new int[]{6, 30, 58, 86, ByteCodes.fmod, 142, ByteCodes.tableswitch}};
    private static final int[][] TYPE_INFO_COORDINATES = {new int[]{8, 0}, new int[]{8, 1}, new int[]{8, 2}, new int[]{8, 3}, new int[]{8, 4}, new int[]{8, 5}, new int[]{8, 7}, new int[]{8, 8}, new int[]{7, 8}, new int[]{5, 8}, new int[]{4, 8}, new int[]{3, 8}, new int[]{2, 8}, new int[]{1, 8}, new int[]{0, 8}};

    private MatrixUtil() {
    }

    static void clearMatrix(ByteMatrix matrix) {
        matrix.clear((byte) -1);
    }

    static void buildMatrix(BitArray dataBits, ErrorCorrectionLevel ecLevel, Version version, int maskPattern, ByteMatrix matrix) throws WriterException {
        clearMatrix(matrix);
        embedBasicPatterns(version, matrix);
        embedTypeInfo(ecLevel, maskPattern, matrix);
        maybeEmbedVersionInfo(version, matrix);
        embedDataBits(dataBits, maskPattern, matrix);
    }

    static void embedBasicPatterns(Version version, ByteMatrix matrix) throws WriterException {
        embedPositionDetectionPatternsAndSeparators(matrix);
        embedDarkDotAtLeftBottomCorner(matrix);
        maybeEmbedPositionAdjustmentPatterns(version, matrix);
        embedTimingPatterns(matrix);
    }

    static void embedTypeInfo(ErrorCorrectionLevel ecLevel, int maskPattern, ByteMatrix matrix) throws WriterException {
        int x2;
        int y2;
        BitArray typeInfoBits = new BitArray();
        makeTypeInfoBits(ecLevel, maskPattern, typeInfoBits);
        for (int i = 0; i < typeInfoBits.getSize(); i++) {
            boolean bit = typeInfoBits.get((typeInfoBits.getSize() - 1) - i);
            int[] coordinates = TYPE_INFO_COORDINATES[i];
            int x1 = coordinates[0];
            int y1 = coordinates[1];
            matrix.set(x1, y1, bit);
            if (i < 8) {
                x2 = (matrix.getWidth() - i) - 1;
                y2 = 8;
            } else {
                x2 = 8;
                y2 = (matrix.getHeight() - 7) + (i - 8);
            }
            matrix.set(x2, y2, bit);
        }
    }

    static void maybeEmbedVersionInfo(Version version, ByteMatrix matrix) throws WriterException {
        if (version.getVersionNumber() < 7) {
            return;
        }
        BitArray versionInfoBits = new BitArray();
        makeVersionInfoBits(version, versionInfoBits);
        int bitIndex = 17;
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 3; j++) {
                boolean bit = versionInfoBits.get(bitIndex);
                bitIndex--;
                matrix.set(i, (matrix.getHeight() - 11) + j, bit);
                matrix.set((matrix.getHeight() - 11) + j, i, bit);
            }
        }
    }

    static void embedDataBits(BitArray dataBits, int maskPattern, ByteMatrix matrix) throws WriterException {
        boolean bit;
        int bitIndex = 0;
        int direction = -1;
        int x = matrix.getWidth() - 1;
        int y = matrix.getHeight() - 1;
        while (x > 0) {
            if (x == 6) {
                x--;
            }
            while (y >= 0 && y < matrix.getHeight()) {
                for (int i = 0; i < 2; i++) {
                    int xx = x - i;
                    if (isEmpty(matrix.get(xx, y))) {
                        if (bitIndex < dataBits.getSize()) {
                            bit = dataBits.get(bitIndex);
                            bitIndex++;
                        } else {
                            bit = false;
                        }
                        if (maskPattern != -1 && MaskUtil.getDataMaskBit(maskPattern, xx, y)) {
                            bit = !bit;
                        }
                        matrix.set(xx, y, bit);
                    }
                }
                y += direction;
            }
            direction = -direction;
            y += direction;
            x -= 2;
        }
        if (bitIndex != dataBits.getSize()) {
            throw new WriterException("Not all bits consumed: " + bitIndex + DataResource.SEPARATOR + dataBits.getSize());
        }
    }

    static int findMSBSet(int value) {
        return 32 - Integer.numberOfLeadingZeros(value);
    }

    static int calculateBCHCode(int value, int poly) {
        if (poly == 0) {
            throw new IllegalArgumentException("0 polynomial");
        }
        int msbSetInPoly = findMSBSet(poly);
        int value2 = value << (msbSetInPoly - 1);
        while (findMSBSet(value2) >= msbSetInPoly) {
            value2 ^= poly << (findMSBSet(value2) - msbSetInPoly);
        }
        return value2;
    }

    static void makeTypeInfoBits(ErrorCorrectionLevel ecLevel, int maskPattern, BitArray bits) throws WriterException {
        if (!QRCode.isValidMaskPattern(maskPattern)) {
            throw new WriterException("Invalid mask pattern");
        }
        int typeInfo = (ecLevel.getBits() << 3) | maskPattern;
        bits.appendBits(typeInfo, 5);
        int bchCode = calculateBCHCode(typeInfo, TYPE_INFO_POLY);
        bits.appendBits(bchCode, 10);
        BitArray maskBits = new BitArray();
        maskBits.appendBits(TYPE_INFO_MASK_PATTERN, 15);
        bits.xor(maskBits);
        if (bits.getSize() != 15) {
            throw new WriterException("should not happen but we got: " + bits.getSize());
        }
    }

    static void makeVersionInfoBits(Version version, BitArray bits) throws WriterException {
        bits.appendBits(version.getVersionNumber(), 6);
        int bchCode = calculateBCHCode(version.getVersionNumber(), VERSION_INFO_POLY);
        bits.appendBits(bchCode, 12);
        if (bits.getSize() != 18) {
            throw new WriterException("should not happen but we got: " + bits.getSize());
        }
    }

    private static boolean isEmpty(int value) {
        return value == -1;
    }

    private static void embedTimingPatterns(ByteMatrix matrix) {
        for (int i = 8; i < matrix.getWidth() - 8; i++) {
            int bit = (i + 1) % 2;
            if (isEmpty(matrix.get(i, 6))) {
                matrix.set(i, 6, bit);
            }
            if (isEmpty(matrix.get(6, i))) {
                matrix.set(6, i, bit);
            }
        }
    }

    private static void embedDarkDotAtLeftBottomCorner(ByteMatrix matrix) throws WriterException {
        if (matrix.get(8, matrix.getHeight() - 8) == 0) {
            throw new WriterException();
        }
        matrix.set(8, matrix.getHeight() - 8, 1);
    }

    private static void embedHorizontalSeparationPattern(int xStart, int yStart, ByteMatrix matrix) throws WriterException {
        for (int x = 0; x < 8; x++) {
            if (!isEmpty(matrix.get(xStart + x, yStart))) {
                throw new WriterException();
            }
            matrix.set(xStart + x, yStart, 0);
        }
    }

    private static void embedVerticalSeparationPattern(int xStart, int yStart, ByteMatrix matrix) throws WriterException {
        for (int y = 0; y < 7; y++) {
            if (!isEmpty(matrix.get(xStart, yStart + y))) {
                throw new WriterException();
            }
            matrix.set(xStart, yStart + y, 0);
        }
    }

    private static void embedPositionAdjustmentPattern(int xStart, int yStart, ByteMatrix matrix) {
        for (int y = 0; y < 5; y++) {
            int[] patternY = POSITION_ADJUSTMENT_PATTERN[y];
            for (int x = 0; x < 5; x++) {
                matrix.set(xStart + x, yStart + y, patternY[x]);
            }
        }
    }

    private static void embedPositionDetectionPattern(int xStart, int yStart, ByteMatrix matrix) {
        for (int y = 0; y < 7; y++) {
            int[] patternY = POSITION_DETECTION_PATTERN[y];
            for (int x = 0; x < 7; x++) {
                matrix.set(xStart + x, yStart + y, patternY[x]);
            }
        }
    }

    private static void embedPositionDetectionPatternsAndSeparators(ByteMatrix matrix) throws WriterException {
        int pdpWidth = POSITION_DETECTION_PATTERN[0].length;
        embedPositionDetectionPattern(0, 0, matrix);
        embedPositionDetectionPattern(matrix.getWidth() - pdpWidth, 0, matrix);
        embedPositionDetectionPattern(0, matrix.getWidth() - pdpWidth, matrix);
        embedHorizontalSeparationPattern(0, 7, matrix);
        embedHorizontalSeparationPattern(matrix.getWidth() - 8, 7, matrix);
        embedHorizontalSeparationPattern(0, matrix.getWidth() - 8, matrix);
        embedVerticalSeparationPattern(7, 0, matrix);
        embedVerticalSeparationPattern((matrix.getHeight() - 7) - 1, 0, matrix);
        embedVerticalSeparationPattern(7, matrix.getHeight() - 7, matrix);
    }

    private static void maybeEmbedPositionAdjustmentPatterns(Version version, ByteMatrix matrix) {
        if (version.getVersionNumber() < 2) {
            return;
        }
        int index = version.getVersionNumber() - 1;
        int[] coordinates = POSITION_ADJUSTMENT_PATTERN_COORDINATE_TABLE[index];
        for (int y : coordinates) {
            if (y >= 0) {
                for (int x : coordinates) {
                    if (x >= 0 && isEmpty(matrix.get(x, y))) {
                        embedPositionAdjustmentPattern(x - 2, y - 2, matrix);
                    }
                }
            }
        }
    }
}
