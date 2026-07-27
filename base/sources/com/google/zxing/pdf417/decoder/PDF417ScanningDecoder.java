package com.google.zxing.pdf417.decoder;

import com.google.zxing.ChecksumException;
import com.google.zxing.FormatException;
import com.google.zxing.NotFoundException;
import com.google.zxing.ResultPoint;
import com.google.zxing.common.BitMatrix;
import com.google.zxing.common.DecoderResult;
import com.google.zxing.common.detector.MathUtils;
import com.google.zxing.pdf417.PDF417Common;
import com.google.zxing.pdf417.decoder.ec.ErrorCorrection;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Formatter;
import java.util.List;

/* JADX INFO: loaded from: classes8.dex */
public final class PDF417ScanningDecoder {
    private static final int CODEWORD_SKEW_SIZE = 2;
    private static final int MAX_EC_CODEWORDS = 512;
    private static final int MAX_ERRORS = 3;
    private static final ErrorCorrection errorCorrection = new ErrorCorrection();

    private PDF417ScanningDecoder() {
    }

    public static DecoderResult decode(BitMatrix image, ResultPoint imageTopLeft, ResultPoint imageBottomLeft, ResultPoint imageTopRight, ResultPoint imageBottomRight, int minCodewordWidth, int maxCodewordWidth) throws NotFoundException, ChecksumException, FormatException {
        BoundingBox boundingBox;
        DetectionResultColumn detectionResultColumn;
        BoundingBox boundingBox2;
        DetectionResultColumn detectionResultColumn2;
        int barcodeColumn;
        int startColumn;
        BoundingBox boundingBox3 = new BoundingBox(image, imageTopLeft, imageBottomLeft, imageTopRight, imageBottomRight);
        DetectionResultRowIndicatorColumn leftRowIndicatorColumn = null;
        DetectionResultRowIndicatorColumn rightRowIndicatorColumn = null;
        boolean firstPass = true;
        DetectionResult detectionResult = null;
        while (true) {
            if (imageTopLeft != null) {
                leftRowIndicatorColumn = getRowIndicatorColumn(image, boundingBox3, imageTopLeft, true, minCodewordWidth, maxCodewordWidth);
            }
            if (imageTopRight != null) {
                rightRowIndicatorColumn = getRowIndicatorColumn(image, boundingBox3, imageTopRight, false, minCodewordWidth, maxCodewordWidth);
            }
            DetectionResult detectionResultMerge = merge(leftRowIndicatorColumn, rightRowIndicatorColumn);
            detectionResult = detectionResultMerge;
            if (detectionResultMerge == null) {
                throw NotFoundException.getNotFoundInstance();
            }
            BoundingBox resultBox = detectionResult.getBoundingBox();
            if (!firstPass || resultBox == null || (resultBox.getMinY() >= boundingBox3.getMinY() && resultBox.getMaxY() <= boundingBox3.getMaxY())) {
                break;
            }
            boundingBox3 = resultBox;
            firstPass = false;
        }
        detectionResult.setBoundingBox(boundingBox3);
        boolean z = true;
        int maxBarcodeColumn = detectionResult.getBarcodeColumnCount() + 1;
        boolean z2 = false;
        detectionResult.setDetectionResultColumn(0, leftRowIndicatorColumn);
        detectionResult.setDetectionResultColumn(maxBarcodeColumn, rightRowIndicatorColumn);
        boolean leftToRight = leftRowIndicatorColumn != null;
        int barcodeColumnCount = 1;
        int maxCodewordWidth2 = maxCodewordWidth;
        int minCodewordWidth2 = minCodewordWidth;
        while (barcodeColumnCount <= maxBarcodeColumn) {
            int barcodeColumn2 = leftToRight ? barcodeColumnCount : maxBarcodeColumn - barcodeColumnCount;
            if (detectionResult.getDetectionResultColumn(barcodeColumn2) != null) {
                boundingBox = boundingBox3;
            } else {
                if (barcodeColumn2 == 0 || barcodeColumn2 == maxBarcodeColumn) {
                    detectionResultColumn = new DetectionResultRowIndicatorColumn(boundingBox3, barcodeColumn2 == 0 ? z : z2);
                } else {
                    detectionResultColumn = new DetectionResultColumn(boundingBox3);
                }
                detectionResult.setDetectionResultColumn(barcodeColumn2, detectionResultColumn);
                int previousStartColumn = -1;
                int imageRow = boundingBox3.getMinY();
                while (imageRow <= boundingBox3.getMaxY()) {
                    int startColumn2 = getStartColumn(detectionResult, barcodeColumn2, imageRow, leftToRight);
                    if (startColumn2 >= 0 && startColumn2 <= boundingBox3.getMaxX()) {
                        startColumn = startColumn2;
                    } else if (previousStartColumn == -1) {
                        boundingBox2 = boundingBox3;
                        detectionResultColumn2 = detectionResultColumn;
                        barcodeColumn = barcodeColumn2;
                        imageRow++;
                        detectionResultColumn = detectionResultColumn2;
                        boundingBox3 = boundingBox2;
                        barcodeColumn2 = barcodeColumn;
                    } else {
                        startColumn = previousStartColumn;
                    }
                    int minX = boundingBox3.getMinX();
                    int maxX = boundingBox3.getMaxX();
                    int previousStartColumn2 = previousStartColumn;
                    boundingBox2 = boundingBox3;
                    detectionResultColumn2 = detectionResultColumn;
                    barcodeColumn = barcodeColumn2;
                    Codeword codeword = detectCodeword(image, minX, maxX, leftToRight, startColumn, imageRow, minCodewordWidth2, maxCodewordWidth2);
                    if (codeword != null) {
                        detectionResultColumn2.setCodeword(imageRow, codeword);
                        int previousStartColumn3 = startColumn;
                        minCodewordWidth2 = Math.min(minCodewordWidth2, codeword.getWidth());
                        maxCodewordWidth2 = Math.max(maxCodewordWidth2, codeword.getWidth());
                        previousStartColumn = previousStartColumn3;
                    } else {
                        previousStartColumn = previousStartColumn2;
                    }
                    imageRow++;
                    detectionResultColumn = detectionResultColumn2;
                    boundingBox3 = boundingBox2;
                    barcodeColumn2 = barcodeColumn;
                }
                boundingBox = boundingBox3;
            }
            barcodeColumnCount++;
            boundingBox3 = boundingBox;
            z = true;
            z2 = false;
        }
        return createDecoderResult(detectionResult);
    }

    private static DetectionResult merge(DetectionResultRowIndicatorColumn leftRowIndicatorColumn, DetectionResultRowIndicatorColumn rightRowIndicatorColumn) throws NotFoundException {
        BarcodeMetadata barcodeMetadata;
        if ((leftRowIndicatorColumn == null && rightRowIndicatorColumn == null) || (barcodeMetadata = getBarcodeMetadata(leftRowIndicatorColumn, rightRowIndicatorColumn)) == null) {
            return null;
        }
        BoundingBox boundingBox = BoundingBox.merge(adjustBoundingBox(leftRowIndicatorColumn), adjustBoundingBox(rightRowIndicatorColumn));
        return new DetectionResult(barcodeMetadata, boundingBox);
    }

    private static BoundingBox adjustBoundingBox(DetectionResultRowIndicatorColumn rowIndicatorColumn) throws NotFoundException {
        int[] rowHeights;
        if (rowIndicatorColumn == null || (rowHeights = rowIndicatorColumn.getRowHeights()) == null) {
            return null;
        }
        int maxRowHeight = getMax(rowHeights);
        int missingStartRows = 0;
        for (int rowHeight : rowHeights) {
            missingStartRows += maxRowHeight - rowHeight;
            if (rowHeight > 0) {
                break;
            }
        }
        Codeword[] codewords = rowIndicatorColumn.getCodewords();
        for (int row = 0; missingStartRows > 0 && codewords[row] == null; row++) {
            missingStartRows--;
        }
        int missingEndRows = 0;
        for (int row2 = rowHeights.length - 1; row2 >= 0; row2--) {
            missingEndRows += maxRowHeight - rowHeights[row2];
            if (rowHeights[row2] > 0) {
                break;
            }
        }
        int row3 = codewords.length;
        for (int row4 = row3 - 1; missingEndRows > 0 && codewords[row4] == null; row4--) {
            missingEndRows--;
        }
        return rowIndicatorColumn.getBoundingBox().addMissingRows(missingStartRows, missingEndRows, rowIndicatorColumn.isLeft());
    }

    private static int getMax(int[] values) {
        int maxValue = -1;
        for (int value : values) {
            maxValue = Math.max(maxValue, value);
        }
        return maxValue;
    }

    private static BarcodeMetadata getBarcodeMetadata(DetectionResultRowIndicatorColumn leftRowIndicatorColumn, DetectionResultRowIndicatorColumn rightRowIndicatorColumn) {
        BarcodeMetadata leftBarcodeMetadata;
        BarcodeMetadata rightBarcodeMetadata;
        if (leftRowIndicatorColumn == null || (leftBarcodeMetadata = leftRowIndicatorColumn.getBarcodeMetadata()) == null) {
            if (rightRowIndicatorColumn == null) {
                return null;
            }
            return rightRowIndicatorColumn.getBarcodeMetadata();
        }
        if (rightRowIndicatorColumn == null || (rightBarcodeMetadata = rightRowIndicatorColumn.getBarcodeMetadata()) == null || leftBarcodeMetadata.getColumnCount() == rightBarcodeMetadata.getColumnCount() || leftBarcodeMetadata.getErrorCorrectionLevel() == rightBarcodeMetadata.getErrorCorrectionLevel() || leftBarcodeMetadata.getRowCount() == rightBarcodeMetadata.getRowCount()) {
            return leftBarcodeMetadata;
        }
        return null;
    }

    /* JADX WARN: Incorrect condition in loop: B:10:0x0027 */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    private static com.google.zxing.pdf417.decoder.DetectionResultRowIndicatorColumn getRowIndicatorColumn(com.google.zxing.common.BitMatrix r16, com.google.zxing.pdf417.decoder.BoundingBox r17, com.google.zxing.ResultPoint r18, boolean r19, int r20, int r21) {
        /*
            r8 = r19
            com.google.zxing.pdf417.decoder.DetectionResultRowIndicatorColumn r0 = new com.google.zxing.pdf417.decoder.DetectionResultRowIndicatorColumn
            r9 = r17
            r0.<init>(r9, r8)
            r10 = r0
            r0 = 0
            r1 = 0
            r11 = r0
        Ld:
            r0 = 2
            if (r11 >= r0) goto L5c
            if (r11 != 0) goto L14
            r0 = 1
            goto L15
        L14:
            r0 = -1
        L15:
            r12 = r0
            float r0 = r18.getX()
            int r0 = (int) r0
            float r2 = r18.getY()
            int r2 = (int) r2
            r13 = r0
            r14 = r1
            r15 = r2
        L23:
            int r0 = r17.getMaxY()
            if (r15 > r0) goto L58
            int r0 = r17.getMinY()
            if (r15 < r0) goto L58
            r1 = 0
            int r2 = r16.getWidth()
            r0 = r16
            r3 = r19
            r4 = r13
            r5 = r15
            r6 = r20
            r7 = r21
            com.google.zxing.pdf417.decoder.Codeword r0 = detectCodeword(r0, r1, r2, r3, r4, r5, r6, r7)
            r1 = r14
            r14 = r0
            if (r0 == 0) goto L56
            r10.setCodeword(r15, r14)
            if (r8 == 0) goto L51
            int r0 = r14.getStartX()
            r13 = r0
            goto L56
        L51:
            int r0 = r14.getEndX()
            r13 = r0
        L56:
            int r15 = r15 + r12
            goto L23
        L58:
            int r11 = r11 + 1
            r1 = r14
            goto Ld
        L5c:
            return r10
        */
        throw new UnsupportedOperationException("Method not decompiled: com.google.zxing.pdf417.decoder.PDF417ScanningDecoder.getRowIndicatorColumn(com.google.zxing.common.BitMatrix, com.google.zxing.pdf417.decoder.BoundingBox, com.google.zxing.ResultPoint, boolean, int, int):com.google.zxing.pdf417.decoder.DetectionResultRowIndicatorColumn");
    }

    private static void adjustCodewordCount(DetectionResult detectionResult, BarcodeValue[][] barcodeMatrix) throws NotFoundException {
        BarcodeValue barcodeMatrix01 = barcodeMatrix[0][1];
        int[] numberOfCodewords = barcodeMatrix01.getValue();
        int calculatedNumberOfCodewords = (detectionResult.getBarcodeColumnCount() * detectionResult.getBarcodeRowCount()) - getNumberOfECCodeWords(detectionResult.getBarcodeECLevel());
        if (numberOfCodewords.length == 0) {
            if (calculatedNumberOfCodewords <= 0 || calculatedNumberOfCodewords > 928) {
                throw NotFoundException.getNotFoundInstance();
            }
            barcodeMatrix01.setValue(calculatedNumberOfCodewords);
            return;
        }
        if (numberOfCodewords[0] != calculatedNumberOfCodewords && calculatedNumberOfCodewords > 0 && calculatedNumberOfCodewords <= 928) {
            barcodeMatrix01.setValue(calculatedNumberOfCodewords);
        }
    }

    private static DecoderResult createDecoderResult(DetectionResult detectionResult) throws NotFoundException, ChecksumException, FormatException {
        BarcodeValue[][] barcodeMatrix = createBarcodeMatrix(detectionResult);
        adjustCodewordCount(detectionResult, barcodeMatrix);
        Collection<Integer> erasures = new ArrayList<>();
        int[] codewords = new int[detectionResult.getBarcodeRowCount() * detectionResult.getBarcodeColumnCount()];
        List<int[]> ambiguousIndexValuesList = new ArrayList<>();
        Collection<Integer> ambiguousIndexesList = new ArrayList<>();
        for (int row = 0; row < detectionResult.getBarcodeRowCount(); row++) {
            for (int column = 0; column < detectionResult.getBarcodeColumnCount(); column++) {
                int[] values = barcodeMatrix[row][column + 1].getValue();
                int codewordIndex = (detectionResult.getBarcodeColumnCount() * row) + column;
                if (values.length == 0) {
                    erasures.add(Integer.valueOf(codewordIndex));
                } else if (values.length == 1) {
                    codewords[codewordIndex] = values[0];
                } else {
                    ambiguousIndexesList.add(Integer.valueOf(codewordIndex));
                    ambiguousIndexValuesList.add(values);
                }
            }
        }
        int row2 = ambiguousIndexValuesList.size();
        int[][] ambiguousIndexValues = new int[row2][];
        for (int i = 0; i < ambiguousIndexValues.length; i++) {
            ambiguousIndexValues[i] = ambiguousIndexValuesList.get(i);
        }
        int i2 = detectionResult.getBarcodeECLevel();
        return createDecoderResultFromAmbiguousValues(i2, codewords, PDF417Common.toIntArray(erasures), PDF417Common.toIntArray(ambiguousIndexesList), ambiguousIndexValues);
    }

    /* JADX WARN: Code restructure failed: missing block: B:25:0x0047, code lost:
    
        r1 = r2;
     */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    private static com.google.zxing.common.DecoderResult createDecoderResultFromAmbiguousValues(int r6, int[] r7, int[] r8, int[] r9, int[][] r10) throws com.google.zxing.ChecksumException, com.google.zxing.FormatException {
        /*
            int r0 = r9.length
            int[] r0 = new int[r0]
            r1 = 100
        L5:
            int r2 = r1 + (-1)
            if (r1 <= 0) goto L4e
            r1 = 0
        La:
            int r3 = r0.length
            if (r1 >= r3) goto L1a
            r3 = r9[r1]
            r4 = r10[r1]
            r5 = r0[r1]
            r4 = r4[r5]
            r7[r3] = r4
            int r1 = r1 + 1
            goto La
        L1a:
            com.google.zxing.common.DecoderResult r1 = decodeCodewords(r7, r6, r8)     // Catch: com.google.zxing.ChecksumException -> L1f
            return r1
        L1f:
            r1 = move-exception
            int r1 = r0.length
            if (r1 == 0) goto L49
            r1 = 0
        L24:
            int r3 = r0.length
            if (r1 >= r3) goto L47
            r3 = r0[r1]
            r4 = r10[r1]
            int r4 = r4.length
            int r4 = r4 + (-1)
            if (r3 >= r4) goto L37
            r3 = r0[r1]
            int r3 = r3 + 1
            r0[r1] = r3
            goto L47
        L37:
            r3 = 0
            r0[r1] = r3
            int r3 = r0.length
            int r3 = r3 + (-1)
            if (r1 == r3) goto L42
            int r1 = r1 + 1
            goto L24
        L42:
            com.google.zxing.ChecksumException r3 = com.google.zxing.ChecksumException.getChecksumInstance()
            throw r3
        L47:
            r1 = r2
            goto L5
        L49:
            com.google.zxing.ChecksumException r1 = com.google.zxing.ChecksumException.getChecksumInstance()
            throw r1
        L4e:
            com.google.zxing.ChecksumException r1 = com.google.zxing.ChecksumException.getChecksumInstance()
            throw r1
        */
        throw new UnsupportedOperationException("Method not decompiled: com.google.zxing.pdf417.decoder.PDF417ScanningDecoder.createDecoderResultFromAmbiguousValues(int, int[], int[], int[], int[][]):com.google.zxing.common.DecoderResult");
    }

    private static BarcodeValue[][] createBarcodeMatrix(DetectionResult detectionResult) {
        int rowNumber;
        BarcodeValue[][] barcodeMatrix = (BarcodeValue[][]) Array.newInstance((Class<?>) BarcodeValue.class, detectionResult.getBarcodeRowCount(), detectionResult.getBarcodeColumnCount() + 2);
        for (int row = 0; row < barcodeMatrix.length; row++) {
            for (int column = 0; column < barcodeMatrix[row].length; column++) {
                barcodeMatrix[row][column] = new BarcodeValue();
            }
        }
        int column2 = 0;
        for (DetectionResultColumn detectionResultColumn : detectionResult.getDetectionResultColumns()) {
            if (detectionResultColumn != null) {
                for (Codeword codeword : detectionResultColumn.getCodewords()) {
                    if (codeword != null && (rowNumber = codeword.getRowNumber()) >= 0 && rowNumber < barcodeMatrix.length) {
                        barcodeMatrix[rowNumber][column2].setValue(codeword.getValue());
                    }
                }
            }
            column2++;
        }
        return barcodeMatrix;
    }

    private static boolean isValidBarcodeColumn(DetectionResult detectionResult, int barcodeColumn) {
        return barcodeColumn >= 0 && barcodeColumn <= detectionResult.getBarcodeColumnCount() + 1;
    }

    private static int getStartColumn(DetectionResult detectionResult, int barcodeColumn, int imageRow, boolean leftToRight) {
        int offset = leftToRight ? 1 : -1;
        Codeword codeword = null;
        if (isValidBarcodeColumn(detectionResult, barcodeColumn - offset)) {
            codeword = detectionResult.getDetectionResultColumn(barcodeColumn - offset).getCodeword(imageRow);
        }
        if (codeword != null) {
            return leftToRight ? codeword.getEndX() : codeword.getStartX();
        }
        Codeword codewordNearby = detectionResult.getDetectionResultColumn(barcodeColumn).getCodewordNearby(imageRow);
        Codeword codeword2 = codewordNearby;
        if (codewordNearby != null) {
            return leftToRight ? codeword2.getStartX() : codeword2.getEndX();
        }
        if (isValidBarcodeColumn(detectionResult, barcodeColumn - offset)) {
            codeword2 = detectionResult.getDetectionResultColumn(barcodeColumn - offset).getCodewordNearby(imageRow);
        }
        if (codeword2 != null) {
            return leftToRight ? codeword2.getEndX() : codeword2.getStartX();
        }
        int skippedColumns = 0;
        while (isValidBarcodeColumn(detectionResult, barcodeColumn - offset)) {
            barcodeColumn -= offset;
            for (Codeword previousRowCodeword : detectionResult.getDetectionResultColumn(barcodeColumn).getCodewords()) {
                if (previousRowCodeword != null) {
                    return (leftToRight ? previousRowCodeword.getEndX() : previousRowCodeword.getStartX()) + (offset * skippedColumns * (previousRowCodeword.getEndX() - previousRowCodeword.getStartX()));
                }
            }
            skippedColumns++;
        }
        BoundingBox boundingBox = detectionResult.getBoundingBox();
        return leftToRight ? boundingBox.getMinX() : boundingBox.getMaxX();
    }

    private static Codeword detectCodeword(BitMatrix bitMatrix, int i, int i2, boolean z, int i3, int i4, int i5, int i6) {
        int i7;
        int decodedValue;
        int codeword;
        int iAdjustCodewordStartColumn = adjustCodewordStartColumn(bitMatrix, i, i2, z, i3, i4);
        int[] moduleBitCount = getModuleBitCount(bitMatrix, i, i2, z, iAdjustCodewordStartColumn, i4);
        if (moduleBitCount == null) {
            return null;
        }
        int iSum = MathUtils.sum(moduleBitCount);
        if (z) {
            i7 = iAdjustCodewordStartColumn + iSum;
        } else {
            for (int i8 = 0; i8 < moduleBitCount.length / 2; i8++) {
                int i9 = moduleBitCount[i8];
                moduleBitCount[i8] = moduleBitCount[(moduleBitCount.length - 1) - i8];
                moduleBitCount[(moduleBitCount.length - 1) - i8] = i9;
            }
            iAdjustCodewordStartColumn -= iSum;
            i7 = iAdjustCodewordStartColumn;
        }
        if (!checkCodewordSkew(iSum, i5, i6) || (codeword = PDF417Common.getCodeword((decodedValue = PDF417CodewordDecoder.getDecodedValue(moduleBitCount)))) == -1) {
            return null;
        }
        return new Codeword(iAdjustCodewordStartColumn, i7, getCodewordBucketNumber(decodedValue), codeword);
    }

    /* JADX WARN: Removed duplicated region for block: B:11:0x0014  */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    private static int[] getModuleBitCount(com.google.zxing.common.BitMatrix r8, int r9, int r10, boolean r11, int r12, int r13) {
        /*
            r0 = r12
            r1 = 8
            int[] r2 = new int[r1]
            r3 = 0
            r4 = 1
            if (r11 == 0) goto Lb
            r5 = r4
            goto Lc
        Lb:
            r5 = -1
        Lc:
            r6 = r11
        Ld:
            if (r11 == 0) goto L12
            if (r0 >= r10) goto L2c
            goto L14
        L12:
            if (r0 < r9) goto L2c
        L14:
            if (r3 >= r1) goto L2c
            boolean r7 = r8.get(r0, r13)
            if (r7 != r6) goto L23
            r7 = r2[r3]
            int r7 = r7 + r4
            r2[r3] = r7
            int r0 = r0 + r5
            goto Ld
        L23:
            int r3 = r3 + 1
            if (r6 != 0) goto L29
            r7 = r4
            goto L2a
        L29:
            r7 = 0
        L2a:
            r6 = r7
            goto Ld
        L2c:
            if (r3 == r1) goto L3b
            if (r11 == 0) goto L32
            r1 = r10
            goto L33
        L32:
            r1 = r9
        L33:
            if (r0 != r1) goto L39
            r1 = 7
            if (r3 != r1) goto L39
            goto L3b
        L39:
            r1 = 0
            return r1
        L3b:
            return r2
        */
        throw new UnsupportedOperationException("Method not decompiled: com.google.zxing.pdf417.decoder.PDF417ScanningDecoder.getModuleBitCount(com.google.zxing.common.BitMatrix, int, int, boolean, int, int):int[]");
    }

    private static int getNumberOfECCodeWords(int barcodeECLevel) {
        return 2 << barcodeECLevel;
    }

    /* JADX WARN: Removed duplicated region for block: B:13:0x0012  */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    private static int adjustCodewordStartColumn(com.google.zxing.common.BitMatrix r6, int r7, int r8, boolean r9, int r10, int r11) {
        /*
            r0 = r10
            r1 = 1
            if (r9 == 0) goto L6
            r2 = -1
            goto L7
        L6:
            r2 = r1
        L7:
            r3 = 0
        L8:
            r4 = 2
            if (r3 >= r4) goto L2d
        Lb:
            if (r9 == 0) goto L10
            if (r0 < r7) goto L23
            goto L12
        L10:
            if (r0 >= r8) goto L23
        L12:
            boolean r5 = r6.get(r0, r11)
            if (r9 != r5) goto L23
            int r5 = r10 - r0
            int r5 = java.lang.Math.abs(r5)
            if (r5 <= r4) goto L21
            return r10
        L21:
            int r0 = r0 + r2
            goto Lb
        L23:
            int r2 = -r2
            if (r9 != 0) goto L28
            r4 = r1
            goto L29
        L28:
            r4 = 0
        L29:
            r9 = r4
            int r3 = r3 + 1
            goto L8
        L2d:
            return r0
        */
        throw new UnsupportedOperationException("Method not decompiled: com.google.zxing.pdf417.decoder.PDF417ScanningDecoder.adjustCodewordStartColumn(com.google.zxing.common.BitMatrix, int, int, boolean, int, int):int");
    }

    private static boolean checkCodewordSkew(int codewordSize, int minCodewordWidth, int maxCodewordWidth) {
        return minCodewordWidth + (-2) <= codewordSize && codewordSize <= maxCodewordWidth + 2;
    }

    private static DecoderResult decodeCodewords(int[] codewords, int ecLevel, int[] erasures) throws ChecksumException, FormatException {
        if (codewords.length == 0) {
            throw FormatException.getFormatInstance();
        }
        int numECCodewords = 1 << (ecLevel + 1);
        int correctedErrorsCount = correctErrors(codewords, erasures, numECCodewords);
        verifyCodewordCount(codewords, numECCodewords);
        DecoderResult decoderResult = DecodedBitStreamParser.decode(codewords, String.valueOf(ecLevel));
        decoderResult.setErrorsCorrected(Integer.valueOf(correctedErrorsCount));
        decoderResult.setErasures(Integer.valueOf(erasures.length));
        return decoderResult;
    }

    private static int correctErrors(int[] codewords, int[] erasures, int numECCodewords) throws ChecksumException {
        if ((erasures != null && erasures.length > (numECCodewords / 2) + 3) || numECCodewords < 0 || numECCodewords > 512) {
            throw ChecksumException.getChecksumInstance();
        }
        return errorCorrection.decode(codewords, numECCodewords, erasures);
    }

    private static void verifyCodewordCount(int[] codewords, int numECCodewords) throws FormatException {
        if (codewords.length < 4) {
            throw FormatException.getFormatInstance();
        }
        int numberOfCodewords = codewords[0];
        if (numberOfCodewords > codewords.length) {
            throw FormatException.getFormatInstance();
        }
        if (numberOfCodewords == 0) {
            if (numECCodewords < codewords.length) {
                codewords[0] = codewords.length - numECCodewords;
                return;
            }
            throw FormatException.getFormatInstance();
        }
    }

    private static int[] getBitCountForCodeword(int codeword) {
        int[] result = new int[8];
        int previousValue = 0;
        int i = 7;
        while (true) {
            if ((codeword & 1) != previousValue) {
                previousValue = codeword & 1;
                i--;
                if (i < 0) {
                    return result;
                }
            }
            result[i] = result[i] + 1;
            codeword >>= 1;
        }
    }

    private static int getCodewordBucketNumber(int codeword) {
        return getCodewordBucketNumber(getBitCountForCodeword(codeword));
    }

    private static int getCodewordBucketNumber(int[] moduleBitCount) {
        return ((((moduleBitCount[0] - moduleBitCount[2]) + moduleBitCount[4]) - moduleBitCount[6]) + 9) % 9;
    }

    public static String toString(BarcodeValue[][] barcodeMatrix) {
        Formatter formatter = new Formatter();
        for (int row = 0; row < barcodeMatrix.length; row++) {
            try {
                formatter.format("Row %2d: ", Integer.valueOf(row));
                for (int column = 0; column < barcodeMatrix[row].length; column++) {
                    BarcodeValue barcodeValue = barcodeMatrix[row][column];
                    if (barcodeValue.getValue().length == 0) {
                        formatter.format("        ", null);
                    } else {
                        formatter.format("%4d(%2d)", Integer.valueOf(barcodeValue.getValue()[0]), barcodeValue.getConfidence(barcodeValue.getValue()[0]));
                    }
                }
                formatter.format("%n", new Object[0]);
            } catch (Throwable th) {
                try {
                    throw th;
                } catch (Throwable th2) {
                    try {
                        formatter.close();
                    } catch (Throwable th3) {
                        th.addSuppressed(th3);
                    }
                    throw th2;
                }
            }
        }
        String string = formatter.toString();
        formatter.close();
        return string;
    }
}
