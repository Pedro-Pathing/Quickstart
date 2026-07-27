package com.google.zxing.multi.qrcode.detector;

import com.google.zxing.DecodeHintType;
import com.google.zxing.NotFoundException;
import com.google.zxing.ResultPoint;
import com.google.zxing.ResultPointCallback;
import com.google.zxing.common.BitMatrix;
import com.google.zxing.qrcode.detector.FinderPattern;
import com.google.zxing.qrcode.detector.FinderPatternFinder;
import com.google.zxing.qrcode.detector.FinderPatternInfo;
import com.qualcomm.hardware.lynx.LynxServoController;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Map;

/* JADX INFO: loaded from: classes8.dex */
public final class MultiFinderPatternFinder extends FinderPatternFinder {
    private static final float DIFF_MODSIZE_CUTOFF = 0.5f;
    private static final float DIFF_MODSIZE_CUTOFF_PERCENT = 0.05f;
    private static final float MAX_MODULE_COUNT_PER_EDGE = 180.0f;
    private static final float MIN_MODULE_COUNT_PER_EDGE = 9.0f;
    private static final FinderPatternInfo[] EMPTY_RESULT_ARRAY = new FinderPatternInfo[0];
    private static final FinderPattern[] EMPTY_FP_ARRAY = new FinderPattern[0];
    private static final FinderPattern[][] EMPTY_FP_2D_ARRAY = new FinderPattern[0][];

    private static final class ModuleSizeComparator implements Serializable, Comparator<FinderPattern> {
        private ModuleSizeComparator() {
        }

        @Override // java.util.Comparator
        public int compare(FinderPattern center1, FinderPattern center2) {
            float value = center2.getEstimatedModuleSize() - center1.getEstimatedModuleSize();
            if (value < LynxServoController.apiPositionFirst) {
                return -1;
            }
            return ((double) value) > LynxServoController.apiPositionFirst ? 1 : 0;
        }
    }

    public MultiFinderPatternFinder(BitMatrix image, ResultPointCallback resultPointCallback) {
        super(image, resultPointCallback);
    }

    private FinderPattern[][] selectMultipleBestPatterns() throws NotFoundException {
        boolean z;
        FinderPattern finderPattern;
        List<FinderPattern> possibleCenters;
        boolean z2;
        boolean z3;
        FinderPattern p1;
        List<FinderPattern> possibleCenters2;
        boolean z4;
        FinderPattern finderPattern2;
        boolean z5;
        FinderPattern p12;
        List<FinderPattern> possibleCenters3;
        FinderPattern p2;
        FinderPattern finderPattern3;
        float vModSize12;
        List<FinderPattern> possibleCenters4 = getPossibleCenters();
        List<FinderPattern> possibleCenters5 = possibleCenters4;
        int size = possibleCenters4.size();
        if (size < 3) {
            throw NotFoundException.getNotFoundInstance();
        }
        boolean z6 = false;
        boolean z7 = true;
        if (size == 3) {
            return new FinderPattern[][]{(FinderPattern[]) possibleCenters5.toArray(EMPTY_FP_ARRAY)};
        }
        Collections.sort(possibleCenters5, new ModuleSizeComparator());
        List<FinderPattern[]> results = new ArrayList<>();
        int i1 = 0;
        while (i1 < size - 2) {
            FinderPattern finderPattern4 = possibleCenters5.get(i1);
            FinderPattern p13 = finderPattern4;
            if (finderPattern4 == null) {
                z = z6;
                finderPattern = p13;
                possibleCenters = possibleCenters5;
                z2 = z7;
            } else {
                int i2 = i1 + 1;
                while (true) {
                    if (i2 >= size - 1) {
                        z = z6;
                        finderPattern = p13;
                        possibleCenters = possibleCenters5;
                        z2 = z7;
                        break;
                    }
                    FinderPattern finderPattern5 = possibleCenters5.get(i2);
                    FinderPattern p22 = finderPattern5;
                    if (finderPattern5 == null) {
                        z3 = z6;
                        p1 = p13;
                        possibleCenters2 = possibleCenters5;
                        z4 = z7;
                        finderPattern2 = p22;
                    } else {
                        float vModSize122 = (p13.getEstimatedModuleSize() - p22.getEstimatedModuleSize()) / Math.min(p13.getEstimatedModuleSize(), p22.getEstimatedModuleSize());
                        float fAbs = Math.abs(p13.getEstimatedModuleSize() - p22.getEstimatedModuleSize());
                        float f = DIFF_MODSIZE_CUTOFF;
                        if (fAbs > DIFF_MODSIZE_CUTOFF && vModSize122 >= 0.05f) {
                            z = z6;
                            finderPattern = p13;
                            possibleCenters = possibleCenters5;
                            z2 = true;
                            break;
                        }
                        int i3 = i2 + 1;
                        while (true) {
                            if (i3 >= size) {
                                z3 = z6;
                                p1 = p13;
                                possibleCenters2 = possibleCenters5;
                                finderPattern2 = p22;
                                z4 = true;
                                break;
                            }
                            FinderPattern p3 = possibleCenters5.get(i3);
                            if (p3 != null) {
                                float vModSize23 = (p22.getEstimatedModuleSize() - p3.getEstimatedModuleSize()) / Math.min(p22.getEstimatedModuleSize(), p3.getEstimatedModuleSize());
                                if (Math.abs(p22.getEstimatedModuleSize() - p3.getEstimatedModuleSize()) > f && vModSize23 >= 0.05f) {
                                    p1 = p13;
                                    possibleCenters2 = possibleCenters5;
                                    finderPattern2 = p22;
                                    z4 = true;
                                    z3 = false;
                                    break;
                                }
                                z5 = false;
                                FinderPattern[] test = {p13, p22, p3};
                                ResultPoint.orderBestPatterns(test);
                                FinderPatternInfo info = new FinderPatternInfo(test);
                                float dA = ResultPoint.distance(info.getTopLeft(), info.getBottomLeft());
                                float dC = ResultPoint.distance(info.getTopRight(), info.getBottomLeft());
                                possibleCenters3 = possibleCenters5;
                                float dB = ResultPoint.distance(info.getTopLeft(), info.getTopRight());
                                float estimatedModuleCount = (dA + dB) / (p13.getEstimatedModuleSize() * 2.0f);
                                if (estimatedModuleCount > MAX_MODULE_COUNT_PER_EDGE || estimatedModuleCount < MIN_MODULE_COUNT_PER_EDGE) {
                                    p12 = p13;
                                    p2 = p22;
                                    finderPattern3 = p3;
                                    vModSize12 = vModSize122;
                                } else if (Math.abs((dA - dB) / Math.min(dA, dB)) < 0.1f) {
                                    finderPattern3 = p3;
                                    vModSize12 = vModSize122;
                                    p2 = p22;
                                    p12 = p13;
                                    float dCpy = (float) Math.sqrt((((double) dA) * ((double) dA)) + (((double) dB) * ((double) dB)));
                                    if (Math.abs((dC - dCpy) / Math.min(dC, dCpy)) < 0.1f) {
                                        results.add(test);
                                    }
                                } else {
                                    p12 = p13;
                                    p2 = p22;
                                    finderPattern3 = p3;
                                    vModSize12 = vModSize122;
                                }
                            } else {
                                z5 = z6;
                                p12 = p13;
                                possibleCenters3 = possibleCenters5;
                                p2 = p22;
                                finderPattern3 = p3;
                                vModSize12 = vModSize122;
                            }
                            i3++;
                            z6 = z5;
                            possibleCenters5 = possibleCenters3;
                            vModSize122 = vModSize12;
                            p22 = p2;
                            p13 = p12;
                            f = DIFF_MODSIZE_CUTOFF;
                        }
                    }
                    i2++;
                    z7 = z4;
                    z6 = z3;
                    possibleCenters5 = possibleCenters2;
                    p13 = p1;
                }
            }
            i1++;
            z7 = z2;
            z6 = z;
            possibleCenters5 = possibleCenters;
        }
        if (!results.isEmpty()) {
            return (FinderPattern[][]) results.toArray(EMPTY_FP_2D_ARRAY);
        }
        throw NotFoundException.getNotFoundInstance();
    }

    public FinderPatternInfo[] findMulti(Map<DecodeHintType, ?> hints) throws NotFoundException {
        boolean tryHarder = hints != null && hints.containsKey(DecodeHintType.TRY_HARDER);
        BitMatrix image = getImage();
        int maxI = image.getHeight();
        int maxJ = image.getWidth();
        int i = (maxI * 3) / 388;
        int iSkip = i;
        if (i < 3 || tryHarder) {
            iSkip = 3;
        }
        int[] stateCount = new int[5];
        for (int i2 = iSkip - 1; i2 < maxI; i2 += iSkip) {
            doClearCounts(stateCount);
            int currentState = 0;
            for (int j = 0; j < maxJ; j++) {
                if (image.get(j, i2)) {
                    if ((currentState & 1) == 1) {
                        currentState++;
                    }
                    stateCount[currentState] = stateCount[currentState] + 1;
                } else if ((currentState & 1) == 0) {
                    if (currentState == 4) {
                        if (foundPatternCross(stateCount) && handlePossibleCenter(stateCount, i2, j)) {
                            currentState = 0;
                            doClearCounts(stateCount);
                        } else {
                            doShiftCounts2(stateCount);
                            currentState = 3;
                        }
                    } else {
                        currentState++;
                        stateCount[currentState] = stateCount[currentState] + 1;
                    }
                } else {
                    stateCount[currentState] = stateCount[currentState] + 1;
                }
            }
            if (foundPatternCross(stateCount)) {
                handlePossibleCenter(stateCount, i2, maxJ);
            }
        }
        FinderPattern[][] patternInfo = selectMultipleBestPatterns();
        List<FinderPatternInfo> result = new ArrayList<>();
        for (FinderPattern[] pattern : patternInfo) {
            ResultPoint.orderBestPatterns(pattern);
            result.add(new FinderPatternInfo(pattern));
        }
        if (result.isEmpty()) {
            return EMPTY_RESULT_ARRAY;
        }
        return (FinderPatternInfo[]) result.toArray(EMPTY_RESULT_ARRAY);
    }
}
