package com.google.zxing.pdf417.detector;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.DecodeHintType;
import com.google.zxing.NotFoundException;
import com.google.zxing.ResultPoint;
import com.google.zxing.common.BitMatrix;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

/* JADX INFO: loaded from: classes8.dex */
public final class Detector {
    private static final int BARCODE_MIN_HEIGHT = 10;
    private static final float MAX_AVG_VARIANCE = 0.42f;
    private static final float MAX_INDIVIDUAL_VARIANCE = 0.8f;
    private static final int MAX_PATTERN_DRIFT = 5;
    private static final int MAX_PIXEL_DRIFT = 3;
    private static final int ROW_STEP = 5;
    private static final int SKIPPED_ROW_COUNT_MAX = 25;
    private static final int[] INDEXES_START_PATTERN = {0, 4, 1, 5};
    private static final int[] INDEXES_STOP_PATTERN = {6, 2, 7, 3};
    private static final int[] START_PATTERN = {8, 1, 1, 1, 1, 1, 1, 3};
    private static final int[] STOP_PATTERN = {7, 1, 1, 3, 1, 1, 1, 2, 1};

    private Detector() {
    }

    public static PDF417DetectorResult detect(BinaryBitmap image, Map<DecodeHintType, ?> hints, boolean multiple) throws NotFoundException {
        BitMatrix bitMatrix = image.getBlackMatrix();
        List<ResultPoint[]> listDetect = detect(multiple, bitMatrix);
        List<ResultPoint[]> barcodeCoordinates = listDetect;
        if (listDetect.isEmpty()) {
            BitMatrix bitMatrixM139clone = bitMatrix.m139clone();
            bitMatrix = bitMatrixM139clone;
            bitMatrixM139clone.rotate180();
            barcodeCoordinates = detect(multiple, bitMatrix);
        }
        return new PDF417DetectorResult(bitMatrix, barcodeCoordinates);
    }

    private static List<ResultPoint[]> detect(boolean multiple, BitMatrix bitMatrix) {
        List<ResultPoint[]> barcodeCoordinates = new ArrayList<>();
        int row = 0;
        int column = 0;
        boolean foundBarcodeInRow = false;
        while (row < bitMatrix.getHeight()) {
            ResultPoint[] vertices = findVertices(bitMatrix, row, column);
            if (vertices[0] == null && vertices[3] == null) {
                if (!foundBarcodeInRow) {
                    break;
                }
                foundBarcodeInRow = false;
                column = 0;
                for (ResultPoint[] barcodeCoordinate : barcodeCoordinates) {
                    if (barcodeCoordinate[1] != null) {
                        row = (int) Math.max(row, barcodeCoordinate[1].getY());
                    }
                    if (barcodeCoordinate[3] != null) {
                        row = Math.max(row, (int) barcodeCoordinate[3].getY());
                    }
                }
                row += 5;
            } else {
                foundBarcodeInRow = true;
                barcodeCoordinates.add(vertices);
                if (!multiple) {
                    break;
                }
                if (vertices[2] == null) {
                    column = (int) vertices[4].getX();
                    row = (int) vertices[4].getY();
                } else {
                    column = (int) vertices[2].getX();
                    row = (int) vertices[2].getY();
                }
            }
        }
        return barcodeCoordinates;
    }

    private static ResultPoint[] findVertices(BitMatrix matrix, int startRow, int startColumn) {
        int height = matrix.getHeight();
        int width = matrix.getWidth();
        ResultPoint[] result = new ResultPoint[8];
        copyToResult(result, findRowsWithPattern(matrix, height, width, startRow, startColumn, START_PATTERN), INDEXES_START_PATTERN);
        if (result[4] != null) {
            startColumn = (int) result[4].getX();
            startRow = (int) result[4].getY();
        }
        copyToResult(result, findRowsWithPattern(matrix, height, width, startRow, startColumn, STOP_PATTERN), INDEXES_STOP_PATTERN);
        return result;
    }

    private static void copyToResult(ResultPoint[] result, ResultPoint[] tmpResult, int[] destinationIndexes) {
        for (int i = 0; i < destinationIndexes.length; i++) {
            result[destinationIndexes[i]] = tmpResult[i];
        }
    }

    private static ResultPoint[] findRowsWithPattern(BitMatrix bitMatrix, int i, int i2, int i3, int i4, int[] iArr) {
        boolean z;
        int i5;
        int i6;
        ResultPoint[] resultPointArr = new ResultPoint[4];
        int[] iArr2 = new int[iArr.length];
        int i7 = i3;
        while (true) {
            if (i7 >= i) {
                z = false;
                break;
            }
            int[] iArrFindGuardPattern = findGuardPattern(bitMatrix, i4, i7, i2, iArr, iArr2);
            if (iArrFindGuardPattern == null) {
                i7 += 5;
            } else {
                int i8 = i7;
                int[] iArr3 = iArrFindGuardPattern;
                int i9 = i8;
                while (true) {
                    if (i9 <= 0) {
                        i6 = i9;
                        break;
                    }
                    int i10 = i9 - 1;
                    int[] iArrFindGuardPattern2 = findGuardPattern(bitMatrix, i4, i10, i2, iArr, iArr2);
                    if (iArrFindGuardPattern2 != null) {
                        iArr3 = iArrFindGuardPattern2;
                        i9 = i10;
                    } else {
                        i6 = i10 + 1;
                        break;
                    }
                }
                float f = i6;
                resultPointArr[0] = new ResultPoint(iArr3[0], f);
                resultPointArr[1] = new ResultPoint(iArr3[1], f);
                z = true;
                i7 = i6;
            }
        }
        int i11 = i7 + 1;
        if (z) {
            int[] iArr4 = {(int) resultPointArr[0].getX(), (int) resultPointArr[1].getX()};
            int i12 = i11;
            int i13 = 0;
            while (true) {
                if (i12 >= i) {
                    i5 = i13;
                    break;
                }
                i5 = i13;
                int[] iArrFindGuardPattern3 = findGuardPattern(bitMatrix, iArr4[0], i12, i2, iArr, iArr2);
                if (iArrFindGuardPattern3 != null && Math.abs(iArr4[0] - iArrFindGuardPattern3[0]) < 5 && Math.abs(iArr4[1] - iArrFindGuardPattern3[1]) < 5) {
                    iArr4 = iArrFindGuardPattern3;
                    i13 = 0;
                } else {
                    if (i5 > 25) {
                        break;
                    }
                    i13 = i5 + 1;
                }
                i12++;
            }
            i11 = i12 - (i5 + 1);
            float f2 = i11;
            resultPointArr[2] = new ResultPoint(iArr4[0], f2);
            resultPointArr[3] = new ResultPoint(iArr4[1], f2);
        }
        if (i11 - i7 < 10) {
            Arrays.fill(resultPointArr, (Object) null);
        }
        return resultPointArr;
    }

    private static int[] findGuardPattern(BitMatrix matrix, int column, int row, int width, int[] pattern, int[] counters) {
        Arrays.fill(counters, 0, counters.length, 0);
        int patternStart = column;
        int pixelDrift = 0;
        while (matrix.get(patternStart, row) && patternStart > 0) {
            int pixelDrift2 = pixelDrift + 1;
            if (pixelDrift >= 3) {
                break;
            }
            patternStart--;
            pixelDrift = pixelDrift2;
        }
        int x = patternStart;
        int counterPosition = 0;
        int patternLength = pattern.length;
        boolean isWhite = false;
        while (x < width) {
            if (matrix.get(x, row) != isWhite) {
                counters[counterPosition] = counters[counterPosition] + 1;
            } else {
                if (counterPosition != patternLength - 1) {
                    counterPosition++;
                } else if (patternMatchVariance(counters, pattern) >= MAX_AVG_VARIANCE) {
                    patternStart += counters[0] + counters[1];
                    System.arraycopy(counters, 2, counters, 0, counterPosition - 1);
                    counters[counterPosition - 1] = 0;
                    counters[counterPosition] = 0;
                    counterPosition--;
                } else {
                    return new int[]{patternStart, x};
                }
                counters[counterPosition] = 1;
                isWhite = isWhite ? false : true;
            }
            x++;
        }
        if (counterPosition == patternLength - 1 && patternMatchVariance(counters, pattern) < MAX_AVG_VARIANCE) {
            return new int[]{patternStart, x - 1};
        }
        return null;
    }

    private static float patternMatchVariance(int[] counters, int[] pattern) {
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
        float maxIndividualVariance = MAX_INDIVIDUAL_VARIANCE * unitBarWidth;
        float totalVariance = 0.0f;
        for (int x = 0; x < numCounters; x++) {
            int counter = counters[x];
            float scaledPattern = pattern[x] * unitBarWidth;
            float f = ((float) counter) > scaledPattern ? counter - scaledPattern : scaledPattern - counter;
            float variance = f;
            if (f > maxIndividualVariance) {
                return Float.POSITIVE_INFINITY;
            }
            totalVariance += variance;
        }
        return totalVariance / total;
    }
}
