package com.google.zxing.oned;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.ChecksumException;
import com.google.zxing.DecodeHintType;
import com.google.zxing.FormatException;
import com.google.zxing.NotFoundException;
import com.google.zxing.Reader;
import com.google.zxing.ReaderException;
import com.google.zxing.Result;
import com.google.zxing.ResultMetadataType;
import com.google.zxing.ResultPoint;
import com.google.zxing.common.BitArray;
import com.sun.tools.javac.jvm.ByteCodes;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.Map;

/* JADX INFO: loaded from: classes8.dex */
public abstract class OneDReader implements Reader {
    public abstract Result decodeRow(int i, BitArray bitArray, Map<DecodeHintType, ?> map) throws NotFoundException, ChecksumException, FormatException;

    @Override // com.google.zxing.Reader
    public Result decode(BinaryBitmap image) throws NotFoundException, FormatException {
        return decode(image, null);
    }

    @Override // com.google.zxing.Reader
    public Result decode(BinaryBitmap image, Map<DecodeHintType, ?> hints) throws NotFoundException, FormatException {
        try {
            return doDecode(image, hints);
        } catch (NotFoundException nfe) {
            if ((hints != null && hints.containsKey(DecodeHintType.TRY_HARDER)) && image.isRotateSupported()) {
                BinaryBitmap rotatedImage = image.rotateCounterClockwise();
                Result result = doDecode(rotatedImage, hints);
                Map<ResultMetadataType, ?> metadata = result.getResultMetadata();
                int orientation = ByteCodes.ishll;
                if (metadata != null && metadata.containsKey(ResultMetadataType.ORIENTATION)) {
                    orientation = (((Integer) metadata.get(ResultMetadataType.ORIENTATION)).intValue() + ByteCodes.ishll) % 360;
                }
                result.putMetadata(ResultMetadataType.ORIENTATION, Integer.valueOf(orientation));
                ResultPoint[] points = result.getResultPoints();
                if (points != null) {
                    int height = rotatedImage.getHeight();
                    for (int i = 0; i < points.length; i++) {
                        points[i] = new ResultPoint((height - points[i].getY()) - 1.0f, points[i].getX());
                    }
                }
                return result;
            }
            throw nfe;
        }
    }

    @Override // com.google.zxing.Reader
    public void reset() {
    }

    private Result doDecode(BinaryBitmap binaryBitmap, Map<DecodeHintType, ?> map) throws NotFoundException {
        int i;
        int i2;
        int i3;
        int i4;
        int width = binaryBitmap.getWidth();
        int height = binaryBitmap.getHeight();
        BitArray bitArray = new BitArray(width);
        int i5 = 0;
        int i6 = 1;
        boolean z = map != null && map.containsKey(DecodeHintType.TRY_HARDER);
        int iMax = Math.max(1, height >> (z ? 8 : 5));
        if (z) {
            i = height;
        } else {
            i = 15;
        }
        int i7 = height / 2;
        BitArray blackRow = bitArray;
        Map<DecodeHintType, ?> map2 = map;
        int i8 = 0;
        while (i8 < i) {
            int i9 = i8 + 1;
            int i10 = i9 / 2;
            if (((i8 & 1) == 0 ? i6 : i5) == 0) {
                i10 = -i10;
            }
            int i11 = (i10 * iMax) + i7;
            if (i11 < 0 || i11 >= height) {
                break;
            }
            try {
                blackRow = binaryBitmap.getBlackRow(i11, blackRow);
                int i12 = i5;
                while (i12 < 2) {
                    if (i12 == i6) {
                        blackRow.reverse();
                        if (map2 != null && map2.containsKey(DecodeHintType.NEED_RESULT_POINT_CALLBACK)) {
                            EnumMap enumMap = new EnumMap(DecodeHintType.class);
                            enumMap.putAll(map2);
                            enumMap.remove(DecodeHintType.NEED_RESULT_POINT_CALLBACK);
                            map2 = enumMap;
                        }
                    }
                    try {
                        Result resultDecodeRow = decodeRow(i11, blackRow, map2);
                        if (i12 == i6) {
                            try {
                                resultDecodeRow.putMetadata(ResultMetadataType.ORIENTATION, Integer.valueOf(ByteCodes.getfield));
                                ResultPoint[] resultPoints = resultDecodeRow.getResultPoints();
                                if (resultPoints != null) {
                                    i3 = height;
                                    float f = width;
                                    try {
                                        i2 = width;
                                    } catch (ReaderException e) {
                                        i2 = width;
                                    }
                                    try {
                                        resultPoints[0] = new ResultPoint((f - resultPoints[i5].getX()) - 1.0f, resultPoints[i5].getY());
                                        i4 = 1;
                                        try {
                                            resultPoints[1] = new ResultPoint((f - resultPoints[1].getX()) - 1.0f, resultPoints[1].getY());
                                        } catch (ReaderException e2) {
                                            i12++;
                                            i6 = i4;
                                            height = i3;
                                            width = i2;
                                            i5 = 0;
                                        }
                                    } catch (ReaderException e3) {
                                        i4 = 1;
                                        i12++;
                                        i6 = i4;
                                        height = i3;
                                        width = i2;
                                        i5 = 0;
                                    }
                                }
                            } catch (ReaderException e4) {
                                i2 = width;
                                i3 = height;
                            }
                        }
                        return resultDecodeRow;
                    } catch (ReaderException e5) {
                        i2 = width;
                        i3 = height;
                        i4 = i6;
                    }
                }
            } catch (NotFoundException e6) {
            }
            i8 = i9;
            i6 = i6;
            height = height;
            width = width;
            i5 = 0;
        }
        throw NotFoundException.getNotFoundInstance();
    }

    protected static void recordPattern(BitArray row, int start, int[] counters) throws NotFoundException {
        int numCounters = counters.length;
        Arrays.fill(counters, 0, numCounters, 0);
        int end = row.getSize();
        if (start >= end) {
            throw NotFoundException.getNotFoundInstance();
        }
        boolean isWhite = !row.get(start);
        int counterPosition = 0;
        int i = start;
        while (i < end) {
            if (row.get(i) != isWhite) {
                counters[counterPosition] = counters[counterPosition] + 1;
            } else {
                counterPosition++;
                if (counterPosition == numCounters) {
                    break;
                }
                counters[counterPosition] = 1;
                isWhite = !isWhite;
            }
            i++;
        }
        if (counterPosition != numCounters) {
            if (counterPosition != numCounters - 1 || i != end) {
                throw NotFoundException.getNotFoundInstance();
            }
        }
    }

    protected static void recordPatternInReverse(BitArray row, int start, int[] counters) throws NotFoundException {
        int numTransitionsLeft = counters.length;
        boolean last = row.get(start);
        while (start > 0 && numTransitionsLeft >= 0) {
            start--;
            if (row.get(start) != last) {
                numTransitionsLeft--;
                last = !last;
            }
        }
        if (numTransitionsLeft >= 0) {
            throw NotFoundException.getNotFoundInstance();
        }
        recordPattern(row, start + 1, counters);
    }

    protected static float patternMatchVariance(int[] counters, int[] pattern, float maxIndividualVariance) {
        int numCounters = counters.length;
        int total = 0;
        int patternLength = 0;
        for (int i = 0; i < numCounters; i++) {
            total += counters[i];
            patternLength += pattern[i];
        }
        if (total < patternLength) {
            return Float.POSITIVE_INFINITY;
        }
        float unitBarWidth = total / patternLength;
        float maxIndividualVariance2 = maxIndividualVariance * unitBarWidth;
        float totalVariance = 0.0f;
        for (int x = 0; x < numCounters; x++) {
            int counter = counters[x];
            float scaledPattern = pattern[x] * unitBarWidth;
            float f = ((float) counter) > scaledPattern ? counter - scaledPattern : scaledPattern - counter;
            float variance = f;
            if (f > maxIndividualVariance2) {
                return Float.POSITIVE_INFINITY;
            }
            totalVariance += variance;
        }
        return totalVariance / total;
    }
}
