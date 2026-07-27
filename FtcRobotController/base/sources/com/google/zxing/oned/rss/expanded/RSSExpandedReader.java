package com.google.zxing.oned.rss.expanded;

import com.google.zxing.BarcodeFormat;
import com.google.zxing.DecodeHintType;
import com.google.zxing.FormatException;
import com.google.zxing.NotFoundException;
import com.google.zxing.Result;
import com.google.zxing.ResultPoint;
import com.google.zxing.common.BitArray;
import com.google.zxing.common.detector.MathUtils;
import com.google.zxing.oned.rss.AbstractRSSReader;
import com.google.zxing.oned.rss.DataCharacter;
import com.google.zxing.oned.rss.FinderPattern;
import com.google.zxing.oned.rss.RSSUtils;
import com.google.zxing.oned.rss.expanded.decoders.AbstractExpandedDecoder;
import com.sun.tools.javac.jvm.ByteCodes;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

/* JADX INFO: loaded from: classes8.dex */
public final class RSSExpandedReader extends AbstractRSSReader {
    private static final int FINDER_PAT_A = 0;
    private static final int FINDER_PAT_B = 1;
    private static final int FINDER_PAT_C = 2;
    private static final int FINDER_PAT_D = 3;
    private static final int FINDER_PAT_E = 4;
    private static final int FINDER_PAT_F = 5;
    private static final int MAX_PAIRS = 11;
    private final List<ExpandedPair> pairs = new ArrayList(11);
    private final List<ExpandedRow> rows = new ArrayList();
    private final int[] startEnd = new int[2];
    private boolean startFromEven;
    private static final int[] SYMBOL_WIDEST = {7, 5, 4, 3, 1};
    private static final int[] EVEN_TOTAL_SUBSET = {4, 20, 52, 104, 204};
    private static final int[] GSUM = {0, 348, 1388, 2948, 3988};
    private static final int[][] FINDER_PATTERNS = {new int[]{1, 8, 4, 1}, new int[]{3, 6, 4, 1}, new int[]{3, 4, 6, 1}, new int[]{3, 2, 8, 1}, new int[]{2, 6, 5, 1}, new int[]{2, 2, 9, 1}};
    private static final int[][] WEIGHTS = {new int[]{1, 3, 9, 27, 81, 32, 96, 77}, new int[]{20, 60, ByteCodes.getfield, 118, 143, 7, 21, 63}, new int[]{ByteCodes.anewarray, 145, 13, 39, 117, 140, 209, 205}, new int[]{ByteCodes.instanceof_, ByteCodes.ifgt, 49, 147, 19, 57, ByteCodes.lookupswitch, 91}, new int[]{62, ByteCodes.invokedynamic, 136, ByteCodes.multianewarray, ByteCodes.ret, 85, 44, 132}, new int[]{ByteCodes.invokeinterface, 133, ByteCodes.newarray, 142, 4, 12, 36, 108}, new int[]{ByteCodes.lmod, 128, 173, 97, 80, 29, 87, 50}, new int[]{150, 28, 84, 41, 123, ByteCodes.ifle, 52, ByteCodes.ifge}, new int[]{46, 138, ByteCodes.ByteCodeCount, ByteCodes.new_, 139, 206, ByteCodes.wide, ByteCodes.if_acmpne}, new int[]{76, 17, 51, 153, 37, 111, 122, 155}, new int[]{43, 129, ByteCodes.areturn, 106, 107, 110, 119, 146}, new int[]{16, 48, 144, 10, 30, 90, 59, ByteCodes.return_}, new int[]{109, 116, 137, 200, ByteCodes.getstatic, 112, 125, ByteCodes.if_icmple}, new int[]{70, 210, 208, ByteCodes.breakpoint, ByteCodes.invokestatic, 130, ByteCodes.putstatic, 115}, new int[]{134, ByteCodes.athrow, 151, 31, 93, 68, 204, ByteCodes.arraylength}, new int[]{148, 22, 66, ByteCodes.if_acmp_null, ByteCodes.ireturn, 94, 71, 2}, new int[]{6, 18, 54, ByteCodes.if_icmpge, 64, ByteCodes.checkcast, 154, 40}, new int[]{120, 149, 25, 75, 14, 42, 126, ByteCodes.goto_}, new int[]{79, 26, 78, 23, 69, 207, ByteCodes.if_acmp_nonnull, ByteCodes.dreturn}, new int[]{103, 98, 83, 38, ByteCodes.fmod, 131, ByteCodes.invokevirtual, 124}, new int[]{ByteCodes.if_icmplt, 61, ByteCodes.invokespecial, 127, ByteCodes.tableswitch, 88, 53, ByteCodes.if_icmpeq}, new int[]{55, ByteCodes.if_acmpeq, 73, 8, 24, 72, 5, 15}, new int[]{45, 135, ByteCodes.monitorenter, ByteCodes.if_icmpne, 58, ByteCodes.freturn, 100, 89}};
    private static final int[][] FINDER_PATTERN_SEQUENCES = {new int[]{0, 0}, new int[]{0, 1, 1}, new int[]{0, 2, 1, 3}, new int[]{0, 4, 1, 3, 2}, new int[]{0, 4, 1, 3, 3, 5}, new int[]{0, 4, 1, 3, 4, 5, 5}, new int[]{0, 0, 1, 1, 2, 2, 3, 3}, new int[]{0, 0, 1, 1, 2, 2, 3, 4, 4}, new int[]{0, 0, 1, 1, 2, 2, 3, 4, 5, 5}, new int[]{0, 0, 1, 1, 2, 3, 3, 4, 4, 5, 5}};

    @Override // com.google.zxing.oned.OneDReader
    public Result decodeRow(int rowNumber, BitArray row, Map<DecodeHintType, ?> hints) throws NotFoundException, FormatException {
        this.pairs.clear();
        this.startFromEven = false;
        try {
            return constructResult(decodeRow2pairs(rowNumber, row));
        } catch (NotFoundException e) {
            this.pairs.clear();
            this.startFromEven = true;
            return constructResult(decodeRow2pairs(rowNumber, row));
        }
    }

    @Override // com.google.zxing.oned.OneDReader, com.google.zxing.Reader
    public void reset() {
        this.pairs.clear();
        this.rows.clear();
    }

    List<ExpandedPair> decodeRow2pairs(int rowNumber, BitArray row) throws NotFoundException {
        boolean done = false;
        while (!done) {
            try {
                this.pairs.add(retrieveNextPair(row, this.pairs, rowNumber));
            } catch (NotFoundException nfe) {
                if (this.pairs.isEmpty()) {
                    throw nfe;
                }
                done = true;
            }
        }
        if (checkChecksum()) {
            return this.pairs;
        }
        boolean tryStackedDecode = !this.rows.isEmpty();
        storeRow(rowNumber);
        if (tryStackedDecode) {
            List<ExpandedPair> ps = checkRows(false);
            if (ps != null) {
                return ps;
            }
            List<ExpandedPair> ps2 = checkRows(true);
            if (ps2 != null) {
                return ps2;
            }
        }
        throw NotFoundException.getNotFoundInstance();
    }

    private List<ExpandedPair> checkRows(boolean reverse) throws NotFoundException {
        if (this.rows.size() > 25) {
            this.rows.clear();
            return null;
        }
        this.pairs.clear();
        if (reverse) {
            Collections.reverse(this.rows);
        }
        List<ExpandedPair> ps = null;
        try {
            ps = checkRows(new ArrayList(), 0);
        } catch (NotFoundException e) {
        }
        if (reverse) {
            Collections.reverse(this.rows);
        }
        return ps;
    }

    private List<ExpandedPair> checkRows(List<ExpandedRow> collectedRows, int currentRow) throws NotFoundException {
        for (int i = currentRow; i < this.rows.size(); i++) {
            ExpandedRow row = this.rows.get(i);
            this.pairs.clear();
            for (ExpandedRow collectedRow : collectedRows) {
                this.pairs.addAll(collectedRow.getPairs());
            }
            this.pairs.addAll(row.getPairs());
            if (isValidSequence(this.pairs)) {
                if (checkChecksum()) {
                    return this.pairs;
                }
                List<ExpandedRow> rs = new ArrayList<>(collectedRows);
                rs.add(row);
                try {
                    return checkRows(rs, i + 1);
                } catch (NotFoundException e) {
                }
            }
        }
        throw NotFoundException.getNotFoundInstance();
    }

    private static boolean isValidSequence(List<ExpandedPair> pairs) {
        for (int[] sequence : FINDER_PATTERN_SEQUENCES) {
            if (pairs.size() <= sequence.length) {
                boolean stop = true;
                int j = 0;
                while (true) {
                    if (j >= pairs.size()) {
                        break;
                    }
                    if (pairs.get(j).getFinderPattern().getValue() == sequence[j]) {
                        j++;
                    } else {
                        stop = false;
                        break;
                    }
                }
                if (stop) {
                    return true;
                }
            }
        }
        return false;
    }

    private void storeRow(int rowNumber) {
        int insertPos = 0;
        boolean prevIsSame = false;
        boolean nextIsSame = false;
        while (true) {
            if (insertPos >= this.rows.size()) {
                break;
            }
            ExpandedRow erow = this.rows.get(insertPos);
            if (erow.getRowNumber() > rowNumber) {
                nextIsSame = erow.isEquivalent(this.pairs);
                break;
            } else {
                prevIsSame = erow.isEquivalent(this.pairs);
                insertPos++;
            }
        }
        if (nextIsSame || prevIsSame || isPartialRow(this.pairs, this.rows)) {
            return;
        }
        this.rows.add(insertPos, new ExpandedRow(this.pairs, rowNumber, false));
        removePartialRows(this.pairs, this.rows);
    }

    private static void removePartialRows(Collection<ExpandedPair> pairs, Collection<ExpandedRow> rows) {
        Iterator<ExpandedRow> iterator = rows.iterator();
        while (iterator.hasNext()) {
            ExpandedRow r = iterator.next();
            if (r.getPairs().size() != pairs.size()) {
                boolean allFound = true;
                Iterator<ExpandedPair> it = r.getPairs().iterator();
                while (true) {
                    if (!it.hasNext()) {
                        break;
                    }
                    ExpandedPair p = it.next();
                    if (!pairs.contains(p)) {
                        allFound = false;
                        break;
                    }
                }
                if (allFound) {
                    iterator.remove();
                }
            }
        }
    }

    private static boolean isPartialRow(Iterable<ExpandedPair> pairs, Iterable<ExpandedRow> rows) {
        for (ExpandedRow r : rows) {
            boolean allFound = true;
            Iterator<ExpandedPair> it = pairs.iterator();
            while (true) {
                if (!it.hasNext()) {
                    break;
                }
                ExpandedPair p = it.next();
                boolean found = false;
                Iterator<ExpandedPair> it2 = r.getPairs().iterator();
                while (true) {
                    if (!it2.hasNext()) {
                        break;
                    }
                    ExpandedPair pp = it2.next();
                    if (p.equals(pp)) {
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    allFound = false;
                    break;
                }
            }
            if (allFound) {
                return true;
            }
        }
        return false;
    }

    List<ExpandedRow> getRows() {
        return this.rows;
    }

    static Result constructResult(List<ExpandedPair> pairs) throws NotFoundException, FormatException {
        String resultingString = AbstractExpandedDecoder.createDecoder(BitArrayBuilder.buildBitArray(pairs)).parseInformation();
        ResultPoint[] firstPoints = pairs.get(0).getFinderPattern().getResultPoints();
        ResultPoint[] lastPoints = pairs.get(pairs.size() - 1).getFinderPattern().getResultPoints();
        return new Result(resultingString, null, new ResultPoint[]{firstPoints[0], firstPoints[1], lastPoints[0], lastPoints[1]}, BarcodeFormat.RSS_EXPANDED);
    }

    private boolean checkChecksum() {
        ExpandedPair firstPair = this.pairs.get(0);
        DataCharacter checkCharacter = firstPair.getLeftChar();
        DataCharacter firstCharacter = firstPair.getRightChar();
        if (firstCharacter == null) {
            return false;
        }
        int checksum = firstCharacter.getChecksumPortion();
        int s = 2;
        for (int i = 1; i < this.pairs.size(); i++) {
            ExpandedPair currentPair = this.pairs.get(i);
            checksum += currentPair.getLeftChar().getChecksumPortion();
            s++;
            DataCharacter currentRightChar = currentPair.getRightChar();
            if (currentRightChar != null) {
                checksum += currentRightChar.getChecksumPortion();
                s++;
            }
        }
        return ((s + (-4)) * 211) + (checksum % 211) == checkCharacter.getValue();
    }

    private static int getNextSecondBar(BitArray row, int initialPos) {
        if (row.get(initialPos)) {
            int currentPos = row.getNextUnset(initialPos);
            return row.getNextSet(currentPos);
        }
        int currentPos2 = row.getNextSet(initialPos);
        return row.getNextUnset(currentPos2);
    }

    ExpandedPair retrieveNextPair(BitArray row, List<ExpandedPair> previousPairs, int rowNumber) throws NotFoundException {
        FinderPattern pattern;
        DataCharacter rightChar;
        boolean isOddPattern = previousPairs.size() % 2 == 0;
        if (this.startFromEven) {
            isOddPattern = !isOddPattern;
        }
        boolean keepFinding = true;
        int forcedOffset = -1;
        do {
            findNextPair(row, previousPairs, forcedOffset);
            pattern = parseFoundFinderPattern(row, rowNumber, isOddPattern);
            if (pattern == null) {
                forcedOffset = getNextSecondBar(row, this.startEnd[0]);
            } else {
                keepFinding = false;
            }
        } while (keepFinding);
        DataCharacter leftChar = decodeDataCharacter(row, pattern, isOddPattern, true);
        if (!previousPairs.isEmpty() && previousPairs.get(previousPairs.size() - 1).mustBeLast()) {
            throw NotFoundException.getNotFoundInstance();
        }
        try {
            rightChar = decodeDataCharacter(row, pattern, isOddPattern, false);
        } catch (NotFoundException e) {
            rightChar = null;
        }
        return new ExpandedPair(leftChar, rightChar, pattern);
    }

    private void findNextPair(BitArray row, List<ExpandedPair> previousPairs, int forcedOffset) throws NotFoundException {
        int rowOffset;
        int[] counters = getDecodeFinderCounters();
        counters[0] = 0;
        counters[1] = 0;
        counters[2] = 0;
        counters[3] = 0;
        int width = row.getSize();
        if (forcedOffset >= 0) {
            rowOffset = forcedOffset;
        } else if (!previousPairs.isEmpty()) {
            rowOffset = previousPairs.get(previousPairs.size() - 1).getFinderPattern().getStartEnd()[1];
        } else {
            rowOffset = 0;
        }
        boolean searchingEvenPair = previousPairs.size() % 2 != 0;
        if (this.startFromEven) {
            searchingEvenPair = !searchingEvenPair;
        }
        boolean isWhite = false;
        while (rowOffset < width) {
            boolean z = row.get(rowOffset);
            isWhite = !z;
            if (z) {
                break;
            } else {
                rowOffset++;
            }
        }
        int counterPosition = 0;
        int patternStart = rowOffset;
        for (int x = rowOffset; x < width; x++) {
            if (row.get(x) != isWhite) {
                counters[counterPosition] = counters[counterPosition] + 1;
            } else {
                if (counterPosition == 3) {
                    if (searchingEvenPair) {
                        reverseCounters(counters);
                    }
                    if (isFinderPattern(counters)) {
                        this.startEnd[0] = patternStart;
                        this.startEnd[1] = x;
                        return;
                    }
                    if (searchingEvenPair) {
                        reverseCounters(counters);
                    }
                    patternStart += counters[0] + counters[1];
                    counters[0] = counters[2];
                    counters[1] = counters[3];
                    counters[2] = 0;
                    counters[3] = 0;
                    counterPosition--;
                } else {
                    counterPosition++;
                }
                counters[counterPosition] = 1;
                isWhite = !isWhite;
            }
        }
        throw NotFoundException.getNotFoundInstance();
    }

    private static void reverseCounters(int[] counters) {
        int length = counters.length;
        for (int i = 0; i < length / 2; i++) {
            int tmp = counters[i];
            counters[i] = counters[(length - i) - 1];
            counters[(length - i) - 1] = tmp;
        }
    }

    private FinderPattern parseFoundFinderPattern(BitArray bitArray, int i, boolean z) {
        int i2;
        int i3;
        int i4;
        if (z) {
            int i5 = this.startEnd[0] - 1;
            while (i5 >= 0 && !bitArray.get(i5)) {
                i5--;
            }
            int i6 = i5 + 1;
            i2 = this.startEnd[0] - i6;
            i3 = i6;
            i4 = this.startEnd[1];
        } else {
            int i7 = this.startEnd[0];
            int nextUnset = bitArray.getNextUnset(this.startEnd[1] + 1);
            i2 = nextUnset - this.startEnd[1];
            i3 = i7;
            i4 = nextUnset;
        }
        int[] decodeFinderCounters = getDecodeFinderCounters();
        System.arraycopy(decodeFinderCounters, 0, decodeFinderCounters, 1, decodeFinderCounters.length - 1);
        decodeFinderCounters[0] = i2;
        try {
            return new FinderPattern(parseFinderValue(decodeFinderCounters, FINDER_PATTERNS), new int[]{i3, i4}, i3, i4, i);
        } catch (NotFoundException e) {
            return null;
        }
    }

    DataCharacter decodeDataCharacter(BitArray bitArray, FinderPattern finderPattern, boolean z, boolean z2) throws NotFoundException {
        int[] dataCharacterCounters = getDataCharacterCounters();
        Arrays.fill(dataCharacterCounters, 0);
        if (z2) {
            recordPatternInReverse(bitArray, finderPattern.getStartEnd()[0], dataCharacterCounters);
        } else {
            recordPattern(bitArray, finderPattern.getStartEnd()[1], dataCharacterCounters);
            int i = 0;
            for (int length = dataCharacterCounters.length - 1; i < length; length--) {
                int i2 = dataCharacterCounters[i];
                dataCharacterCounters[i] = dataCharacterCounters[length];
                dataCharacterCounters[length] = i2;
                i++;
            }
        }
        float fSum = MathUtils.sum(dataCharacterCounters) / 17.0f;
        float f = (finderPattern.getStartEnd()[1] - finderPattern.getStartEnd()[0]) / 15.0f;
        if (Math.abs(fSum - f) / f > 0.3f) {
            throw NotFoundException.getNotFoundInstance();
        }
        int[] oddCounts = getOddCounts();
        int[] evenCounts = getEvenCounts();
        float[] oddRoundingErrors = getOddRoundingErrors();
        float[] evenRoundingErrors = getEvenRoundingErrors();
        for (int i3 = 0; i3 < dataCharacterCounters.length; i3++) {
            float f2 = (dataCharacterCounters[i3] * 1.0f) / fSum;
            int i4 = (int) (0.5f + f2);
            if (i4 <= 0) {
                if (f2 < 0.3f) {
                    throw NotFoundException.getNotFoundInstance();
                }
                i4 = 1;
            } else if (i4 > 8) {
                if (f2 > 8.7f) {
                    throw NotFoundException.getNotFoundInstance();
                }
                i4 = 8;
            }
            int i5 = i3 / 2;
            if ((i3 & 1) == 0) {
                oddCounts[i5] = i4;
                oddRoundingErrors[i5] = f2 - i4;
            } else {
                evenCounts[i5] = i4;
                evenRoundingErrors[i5] = f2 - i4;
            }
        }
        adjustOddEvenCounts(17);
        int value = (((finderPattern.getValue() * 4) + (z ? 0 : 2)) + (!z2 ? 1 : 0)) - 1;
        int i6 = 0;
        int i7 = 0;
        for (int length2 = oddCounts.length - 1; length2 >= 0; length2--) {
            if (isNotA1left(finderPattern, z, z2)) {
                i6 += oddCounts[length2] * WEIGHTS[value][length2 * 2];
            }
            i7 += oddCounts[length2];
        }
        int i8 = 0;
        for (int length3 = evenCounts.length - 1; length3 >= 0; length3--) {
            if (isNotA1left(finderPattern, z, z2)) {
                i8 += evenCounts[length3] * WEIGHTS[value][(length3 * 2) + 1];
            }
        }
        int i9 = i6 + i8;
        if ((i7 & 1) != 0 || i7 > 13 || i7 < 4) {
            throw NotFoundException.getNotFoundInstance();
        }
        int i10 = (13 - i7) / 2;
        int i11 = SYMBOL_WIDEST[i10];
        return new DataCharacter((RSSUtils.getRSSvalue(oddCounts, i11, true) * EVEN_TOTAL_SUBSET[i10]) + RSSUtils.getRSSvalue(evenCounts, 9 - i11, false) + GSUM[i10], i9);
    }

    private static boolean isNotA1left(FinderPattern pattern, boolean isOddPattern, boolean leftChar) {
        return (pattern.getValue() == 0 && isOddPattern && leftChar) ? false : true;
    }

    private void adjustOddEvenCounts(int numModules) throws NotFoundException {
        int oddSum = MathUtils.sum(getOddCounts());
        int evenSum = MathUtils.sum(getEvenCounts());
        boolean incrementOdd = false;
        boolean decrementOdd = false;
        if (oddSum > 13) {
            decrementOdd = true;
        } else if (oddSum < 4) {
            incrementOdd = true;
        }
        boolean incrementEven = false;
        boolean decrementEven = false;
        if (evenSum > 13) {
            decrementEven = true;
        } else if (evenSum < 4) {
            incrementEven = true;
        }
        int mismatch = (oddSum + evenSum) - numModules;
        boolean oddParityBad = (oddSum & 1) == 1;
        boolean evenParityBad = (evenSum & 1) == 0;
        switch (mismatch) {
            case -1:
                if (oddParityBad) {
                    if (evenParityBad) {
                        throw NotFoundException.getNotFoundInstance();
                    }
                    incrementOdd = true;
                } else {
                    if (!evenParityBad) {
                        throw NotFoundException.getNotFoundInstance();
                    }
                    incrementEven = true;
                }
                break;
            case 0:
                if (oddParityBad) {
                    if (!evenParityBad) {
                        throw NotFoundException.getNotFoundInstance();
                    }
                    if (oddSum < evenSum) {
                        incrementOdd = true;
                        decrementEven = true;
                    } else {
                        decrementOdd = true;
                        incrementEven = true;
                    }
                } else if (evenParityBad) {
                    throw NotFoundException.getNotFoundInstance();
                }
                break;
            case 1:
                if (oddParityBad) {
                    if (evenParityBad) {
                        throw NotFoundException.getNotFoundInstance();
                    }
                    decrementOdd = true;
                } else {
                    if (!evenParityBad) {
                        throw NotFoundException.getNotFoundInstance();
                    }
                    decrementEven = true;
                }
                break;
            default:
                throw NotFoundException.getNotFoundInstance();
        }
        if (incrementOdd) {
            if (decrementOdd) {
                throw NotFoundException.getNotFoundInstance();
            }
            increment(getOddCounts(), getOddRoundingErrors());
        }
        if (decrementOdd) {
            decrement(getOddCounts(), getOddRoundingErrors());
        }
        if (incrementEven) {
            if (decrementEven) {
                throw NotFoundException.getNotFoundInstance();
            }
            increment(getEvenCounts(), getOddRoundingErrors());
        }
        if (decrementEven) {
            decrement(getEvenCounts(), getEvenRoundingErrors());
        }
    }
}
