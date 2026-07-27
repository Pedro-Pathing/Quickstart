package com.google.zxing.oned;

import com.google.zxing.BarcodeFormat;
import com.google.zxing.DecodeHintType;
import com.google.zxing.FormatException;
import com.google.zxing.NotFoundException;
import com.google.zxing.Result;
import com.google.zxing.ResultPoint;
import com.google.zxing.common.BitArray;
import java.util.Map;

/* JADX INFO: loaded from: classes8.dex */
public final class ITFReader extends OneDReader {
    private static final float MAX_AVG_VARIANCE = 0.38f;
    private static final float MAX_INDIVIDUAL_VARIANCE = 0.5f;
    private static final int N = 1;
    private static final int W = 3;
    private static final int w = 2;
    private int narrowLineWidth = -1;
    private static final int[] DEFAULT_ALLOWED_LENGTHS = {6, 8, 10, 12, 14};
    private static final int[] START_PATTERN = {1, 1, 1, 1};
    private static final int[][] END_PATTERN_REVERSED = {new int[]{1, 1, 2}, new int[]{1, 1, 3}};
    private static final int[][] PATTERNS = {new int[]{1, 1, 2, 2, 1}, new int[]{2, 1, 1, 1, 2}, new int[]{1, 2, 1, 1, 2}, new int[]{2, 2, 1, 1, 1}, new int[]{1, 1, 2, 1, 2}, new int[]{2, 1, 2, 1, 1}, new int[]{1, 2, 2, 1, 1}, new int[]{1, 1, 1, 2, 2}, new int[]{2, 1, 1, 2, 1}, new int[]{1, 2, 1, 2, 1}, new int[]{1, 1, 3, 3, 1}, new int[]{3, 1, 1, 1, 3}, new int[]{1, 3, 1, 1, 3}, new int[]{3, 3, 1, 1, 1}, new int[]{1, 1, 3, 1, 3}, new int[]{3, 1, 3, 1, 1}, new int[]{1, 3, 3, 1, 1}, new int[]{1, 1, 1, 3, 3}, new int[]{3, 1, 1, 3, 1}, new int[]{1, 3, 1, 3, 1}};

    @Override // com.google.zxing.oned.OneDReader
    public Result decodeRow(int rowNumber, BitArray row, Map<DecodeHintType, ?> hints) throws NotFoundException, FormatException {
        int[] startRange = decodeStart(row);
        int[] endRange = decodeEnd(row);
        StringBuilder result = new StringBuilder(20);
        decodeMiddle(row, startRange[1], endRange[0], result);
        String resultString = result.toString();
        int[] allowedLengths = null;
        if (hints != null) {
            int[] allowedLengths2 = (int[]) hints.get(DecodeHintType.ALLOWED_LENGTHS);
            allowedLengths = allowedLengths2;
        }
        if (allowedLengths == null) {
            allowedLengths = DEFAULT_ALLOWED_LENGTHS;
        }
        int length = resultString.length();
        boolean lengthOK = false;
        int maxAllowedLength = 0;
        int length2 = allowedLengths.length;
        int i = 0;
        while (true) {
            if (i >= length2) {
                break;
            }
            int allowedLength = allowedLengths[i];
            if (length == allowedLength) {
                lengthOK = true;
                break;
            }
            if (allowedLength > maxAllowedLength) {
                maxAllowedLength = allowedLength;
            }
            i++;
        }
        if (!lengthOK && length > maxAllowedLength) {
            lengthOK = true;
        }
        if (!lengthOK) {
            throw FormatException.getFormatInstance();
        }
        return new Result(resultString, null, new ResultPoint[]{new ResultPoint(startRange[1], rowNumber), new ResultPoint(endRange[0], rowNumber)}, BarcodeFormat.ITF);
    }

    private static void decodeMiddle(BitArray row, int payloadStart, int payloadEnd, StringBuilder resultString) throws NotFoundException {
        int[] counterDigitPair = new int[10];
        int[] counterBlack = new int[5];
        int[] counterWhite = new int[5];
        while (payloadStart < payloadEnd) {
            recordPattern(row, payloadStart, counterDigitPair);
            for (int k = 0; k < 5; k++) {
                int twoK = k * 2;
                counterBlack[k] = counterDigitPair[twoK];
                counterWhite[k] = counterDigitPair[twoK + 1];
            }
            int bestMatch = decodeDigit(counterBlack);
            resultString.append((char) (bestMatch + 48));
            int bestMatch2 = decodeDigit(counterWhite);
            resultString.append((char) (bestMatch2 + 48));
            for (int i = 0; i < 10; i++) {
                int counterDigit = counterDigitPair[i];
                payloadStart += counterDigit;
            }
        }
    }

    private int[] decodeStart(BitArray row) throws NotFoundException {
        int endStart = skipWhiteSpace(row);
        int[] startPattern = findGuardPattern(row, endStart, START_PATTERN);
        this.narrowLineWidth = (startPattern[1] - startPattern[0]) / 4;
        validateQuietZone(row, startPattern[0]);
        return startPattern;
    }

    private void validateQuietZone(BitArray row, int startPattern) throws NotFoundException {
        int quietCount = Math.min(this.narrowLineWidth * 10, startPattern);
        for (int i = startPattern - 1; quietCount > 0 && i >= 0 && !row.get(i); i--) {
            quietCount--;
        }
        if (quietCount != 0) {
            throw NotFoundException.getNotFoundInstance();
        }
    }

    private static int skipWhiteSpace(BitArray row) throws NotFoundException {
        int width = row.getSize();
        int endStart = row.getNextSet(0);
        if (endStart != width) {
            return endStart;
        }
        throw NotFoundException.getNotFoundInstance();
    }

    private int[] decodeEnd(BitArray bitArray) throws NotFoundException {
        int[] iArrFindGuardPattern;
        bitArray.reverse();
        try {
            int iSkipWhiteSpace = skipWhiteSpace(bitArray);
            try {
                iArrFindGuardPattern = findGuardPattern(bitArray, iSkipWhiteSpace, END_PATTERN_REVERSED[0]);
            } catch (NotFoundException e) {
                iArrFindGuardPattern = findGuardPattern(bitArray, iSkipWhiteSpace, END_PATTERN_REVERSED[1]);
            }
            validateQuietZone(bitArray, iArrFindGuardPattern[0]);
            int i = iArrFindGuardPattern[0];
            iArrFindGuardPattern[0] = bitArray.getSize() - iArrFindGuardPattern[1];
            iArrFindGuardPattern[1] = bitArray.getSize() - i;
            return iArrFindGuardPattern;
        } finally {
            bitArray.reverse();
        }
    }

    private static int[] findGuardPattern(BitArray row, int rowOffset, int[] pattern) throws NotFoundException {
        int patternLength = pattern.length;
        int[] counters = new int[patternLength];
        int width = row.getSize();
        boolean isWhite = false;
        int counterPosition = 0;
        int patternStart = rowOffset;
        for (int x = rowOffset; x < width; x++) {
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

    private static int decodeDigit(int[] counters) throws NotFoundException {
        float bestVariance = MAX_AVG_VARIANCE;
        int bestMatch = -1;
        int max = PATTERNS.length;
        for (int i = 0; i < max; i++) {
            int[] pattern = PATTERNS[i];
            float variance = patternMatchVariance(counters, pattern, MAX_INDIVIDUAL_VARIANCE);
            if (variance < bestVariance) {
                bestVariance = variance;
                bestMatch = i;
            } else if (variance == bestVariance) {
                bestMatch = -1;
            }
        }
        if (bestMatch >= 0) {
            return bestMatch % 10;
        }
        throw NotFoundException.getNotFoundInstance();
    }
}
