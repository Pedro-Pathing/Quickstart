package com.google.zxing.oned;

import com.google.zxing.BarcodeFormat;
import com.google.zxing.ChecksumException;
import com.google.zxing.DecodeHintType;
import com.google.zxing.FormatException;
import com.google.zxing.NotFoundException;
import com.google.zxing.ReaderException;
import com.google.zxing.Result;
import com.google.zxing.ResultMetadataType;
import com.google.zxing.ResultPoint;
import com.google.zxing.ResultPointCallback;
import com.google.zxing.common.BitArray;
import java.util.Arrays;
import java.util.Map;

/* JADX INFO: loaded from: classes8.dex */
public abstract class UPCEANReader extends OneDReader {
    private static final float MAX_AVG_VARIANCE = 0.48f;
    private static final float MAX_INDIVIDUAL_VARIANCE = 0.7f;
    static final int[] START_END_PATTERN = {1, 1, 1};
    static final int[] MIDDLE_PATTERN = {1, 1, 1, 1, 1};
    static final int[] END_PATTERN = {1, 1, 1, 1, 1, 1};
    static final int[][] L_PATTERNS = {new int[]{3, 2, 1, 1}, new int[]{2, 2, 2, 1}, new int[]{2, 1, 2, 2}, new int[]{1, 4, 1, 1}, new int[]{1, 1, 3, 2}, new int[]{1, 2, 3, 1}, new int[]{1, 1, 1, 4}, new int[]{1, 3, 1, 2}, new int[]{1, 2, 1, 3}, new int[]{3, 1, 1, 2}};
    static final int[][] L_AND_G_PATTERNS = new int[20][];
    private final StringBuilder decodeRowStringBuffer = new StringBuilder(20);
    private final UPCEANExtensionSupport extensionReader = new UPCEANExtensionSupport();
    private final EANManufacturerOrgSupport eanManSupport = new EANManufacturerOrgSupport();

    protected abstract int decodeMiddle(BitArray bitArray, int[] iArr, StringBuilder sb) throws NotFoundException;

    abstract BarcodeFormat getBarcodeFormat();

    static {
        System.arraycopy(L_PATTERNS, 0, L_AND_G_PATTERNS, 0, 10);
        for (int i = 10; i < 20; i++) {
            int[] widths = L_PATTERNS[i - 10];
            int[] reversedWidths = new int[widths.length];
            for (int j = 0; j < widths.length; j++) {
                reversedWidths[j] = widths[(widths.length - j) - 1];
            }
            L_AND_G_PATTERNS[i] = reversedWidths;
        }
    }

    protected UPCEANReader() {
    }

    static int[] findStartGuardPattern(BitArray row) throws NotFoundException {
        boolean foundStart = false;
        int[] startRange = null;
        int nextStart = 0;
        int[] counters = new int[START_END_PATTERN.length];
        while (!foundStart) {
            Arrays.fill(counters, 0, START_END_PATTERN.length, 0);
            int[] iArrFindGuardPattern = findGuardPattern(row, nextStart, false, START_END_PATTERN, counters);
            startRange = iArrFindGuardPattern;
            int start = iArrFindGuardPattern[0];
            nextStart = startRange[1];
            int quietStart = start - (nextStart - start);
            if (quietStart >= 0) {
                foundStart = row.isRange(quietStart, start, false);
            }
        }
        return startRange;
    }

    @Override // com.google.zxing.oned.OneDReader
    public Result decodeRow(int rowNumber, BitArray row, Map<DecodeHintType, ?> hints) throws NotFoundException, ChecksumException, FormatException {
        return decodeRow(rowNumber, row, findStartGuardPattern(row), hints);
    }

    public Result decodeRow(int i, BitArray bitArray, int[] iArr, Map<DecodeHintType, ?> map) throws NotFoundException, ChecksumException, FormatException {
        ResultPointCallback resultPointCallback;
        int length;
        String strLookupCountryIdentifier;
        int[] iArr2 = null;
        if (map == null) {
            resultPointCallback = null;
        } else {
            resultPointCallback = (ResultPointCallback) map.get(DecodeHintType.NEED_RESULT_POINT_CALLBACK);
        }
        boolean z = true;
        if (resultPointCallback != null) {
            resultPointCallback.foundPossibleResultPoint(new ResultPoint((iArr[0] + iArr[1]) / 2.0f, i));
        }
        StringBuilder sb = this.decodeRowStringBuffer;
        sb.setLength(0);
        int iDecodeMiddle = decodeMiddle(bitArray, iArr, sb);
        if (resultPointCallback != null) {
            resultPointCallback.foundPossibleResultPoint(new ResultPoint(iDecodeMiddle, i));
        }
        int[] iArrDecodeEnd = decodeEnd(bitArray, iDecodeMiddle);
        if (resultPointCallback != null) {
            resultPointCallback.foundPossibleResultPoint(new ResultPoint((iArrDecodeEnd[0] + iArrDecodeEnd[1]) / 2.0f, i));
        }
        int i2 = iArrDecodeEnd[1];
        int i3 = (i2 - iArrDecodeEnd[0]) + i2;
        if (i3 >= bitArray.getSize() || !bitArray.isRange(i2, i3, false)) {
            throw NotFoundException.getNotFoundInstance();
        }
        String string = sb.toString();
        if (string.length() < 8) {
            throw FormatException.getFormatInstance();
        }
        if (checkChecksum(string)) {
            BarcodeFormat barcodeFormat = getBarcodeFormat();
            float f = i;
            Result result = new Result(string, null, new ResultPoint[]{new ResultPoint((iArr[1] + iArr[0]) / 2.0f, f), new ResultPoint((iArrDecodeEnd[1] + iArrDecodeEnd[0]) / 2.0f, f)}, barcodeFormat);
            try {
                Result resultDecodeRow = this.extensionReader.decodeRow(i, bitArray, iArrDecodeEnd[1]);
                result.putMetadata(ResultMetadataType.UPC_EAN_EXTENSION, resultDecodeRow.getText());
                result.putAllMetadata(resultDecodeRow.getResultMetadata());
                result.addResultPoints(resultDecodeRow.getResultPoints());
                length = resultDecodeRow.getText().length();
            } catch (ReaderException e) {
                length = 0;
            }
            if (map != null) {
                iArr2 = (int[]) map.get(DecodeHintType.ALLOWED_EAN_EXTENSIONS);
            }
            if (iArr2 != null) {
                int length2 = iArr2.length;
                int i4 = 0;
                while (true) {
                    if (i4 >= length2) {
                        z = false;
                        break;
                    }
                    if (length == iArr2[i4]) {
                        break;
                    }
                    i4++;
                }
                if (!z) {
                    throw NotFoundException.getNotFoundInstance();
                }
            }
            if ((barcodeFormat == BarcodeFormat.EAN_13 || barcodeFormat == BarcodeFormat.UPC_A) && (strLookupCountryIdentifier = this.eanManSupport.lookupCountryIdentifier(string)) != null) {
                result.putMetadata(ResultMetadataType.POSSIBLE_COUNTRY, strLookupCountryIdentifier);
            }
            return result;
        }
        throw ChecksumException.getChecksumInstance();
    }

    boolean checkChecksum(String s) throws FormatException {
        return checkStandardUPCEANChecksum(s);
    }

    static boolean checkStandardUPCEANChecksum(CharSequence s) throws FormatException {
        int length = s.length();
        if (length == 0) {
            return false;
        }
        int check = Character.digit(s.charAt(length - 1), 10);
        return getStandardUPCEANChecksum(s.subSequence(0, length + (-1))) == check;
    }

    static int getStandardUPCEANChecksum(CharSequence charSequence) throws FormatException {
        int length = charSequence.length();
        int i = 0;
        for (int i2 = length - 1; i2 >= 0; i2 -= 2) {
            int iCharAt = charSequence.charAt(i2) - '0';
            if (iCharAt < 0 || iCharAt > 9) {
                throw FormatException.getFormatInstance();
            }
            i += iCharAt;
        }
        int i3 = i * 3;
        for (int i4 = length - 2; i4 >= 0; i4 -= 2) {
            int iCharAt2 = charSequence.charAt(i4) - '0';
            if (iCharAt2 < 0 || iCharAt2 > 9) {
                throw FormatException.getFormatInstance();
            }
            i3 += iCharAt2;
        }
        return (1000 - i3) % 10;
    }

    int[] decodeEnd(BitArray row, int endStart) throws NotFoundException {
        return findGuardPattern(row, endStart, false, START_END_PATTERN);
    }

    static int[] findGuardPattern(BitArray row, int rowOffset, boolean whiteFirst, int[] pattern) throws NotFoundException {
        return findGuardPattern(row, rowOffset, whiteFirst, pattern, new int[pattern.length]);
    }

    private static int[] findGuardPattern(BitArray row, int rowOffset, boolean whiteFirst, int[] pattern, int[] counters) throws NotFoundException {
        int width = row.getSize();
        int rowOffset2 = whiteFirst ? row.getNextUnset(rowOffset) : row.getNextSet(rowOffset);
        int counterPosition = 0;
        int patternStart = rowOffset2;
        int patternLength = pattern.length;
        boolean isWhite = whiteFirst;
        for (int x = rowOffset2; x < width; x++) {
            if (row.get(x) != isWhite) {
                counters[counterPosition] = counters[counterPosition] + 1;
            } else {
                if (counterPosition == patternLength - 1) {
                    if (patternMatchVariance(counters, pattern, MAX_INDIVIDUAL_VARIANCE) < MAX_AVG_VARIANCE) {
                        return new int[]{patternStart, x};
                    }
                    patternStart += counters[0] + counters[1];
                    System.arraycopy(counters, 2, counters, 0, counterPosition - 1);
                    counters[counterPosition - 1] = 0;
                    counters[counterPosition] = 0;
                    counterPosition--;
                } else {
                    counterPosition++;
                }
                counters[counterPosition] = 1;
                isWhite = isWhite ? false : true;
            }
        }
        throw NotFoundException.getNotFoundInstance();
    }

    static int decodeDigit(BitArray row, int[] counters, int rowOffset, int[][] patterns) throws NotFoundException {
        recordPattern(row, rowOffset, counters);
        float bestVariance = MAX_AVG_VARIANCE;
        int bestMatch = -1;
        int max = patterns.length;
        for (int i = 0; i < max; i++) {
            int[] pattern = patterns[i];
            float variance = patternMatchVariance(counters, pattern, MAX_INDIVIDUAL_VARIANCE);
            if (variance < bestVariance) {
                bestVariance = variance;
                bestMatch = i;
            }
        }
        if (bestMatch >= 0) {
            return bestMatch;
        }
        throw NotFoundException.getNotFoundInstance();
    }
}
