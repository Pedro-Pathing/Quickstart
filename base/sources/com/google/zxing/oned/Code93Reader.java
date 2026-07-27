package com.google.zxing.oned;

import com.google.zxing.BarcodeFormat;
import com.google.zxing.ChecksumException;
import com.google.zxing.DecodeHintType;
import com.google.zxing.FormatException;
import com.google.zxing.NotFoundException;
import com.google.zxing.Result;
import com.google.zxing.ResultPoint;
import com.google.zxing.common.BitArray;
import com.sun.tools.javac.jvm.ByteCodes;
import java.util.Arrays;
import java.util.Map;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.videoio.Videoio;

/* JADX INFO: loaded from: classes8.dex */
public final class Code93Reader extends OneDReader {
    static final int ASTERISK_ENCODING;
    static final int[] CHARACTER_ENCODINGS;
    static final String ALPHABET_STRING = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%abcd*";
    private static final char[] ALPHABET = ALPHABET_STRING.toCharArray();
    private final StringBuilder decodeRowResult = new StringBuilder(20);
    private final int[] counters = new int[6];

    static {
        int[] iArr = {ByteCodes.nullchk, 328, 324, 322, 296, 292, 290, 336, ByteCodes.iushrl, 266, Videoio.CAP_PROP_XI_GAIN, Videoio.CAP_PROP_XI_TIMEOUT, Videoio.CAP_PROP_XI_AG_MAX_LIMIT, Videoio.CAP_PROP_XI_TRG_SOURCE, Videoio.CAP_PROP_XI_OFFSET_X, 394, 360, 356, 354, 308, 282, 344, 332, 326, 300, Imgcodecs.IMWRITE_TIFF_ROWSPERSTRIP, Videoio.CAP_PROP_XI_SHUTTER_TYPE, Videoio.CAP_PROP_XI_DECIMATION_PATTERN, Videoio.CAP_PROP_XI_BINNING_VERTICAL, Videoio.CAP_PROP_XI_EXPOSURE_BURST_COUNT, Videoio.CAP_PROP_XI_GPI_SELECTOR, Videoio.CAP_PROP_XI_GPO_MODE, 364, 358, 310, 314, Videoio.CAP_PROP_PVAPI_DECIMATIONHORIZONTAL, Videoio.CAP_PROP_XI_CHIP_TEMP, Videoio.CAP_PROP_XI_COOLING, 458, 366, 374, Videoio.CAP_PROP_XI_BINNING_PATTERN, 294, Videoio.CAP_PROP_XI_IMAGE_IS_COLOR, Videoio.CAP_PROP_XI_CMS, Videoio.CAP_PROP_PVAPI_PIXELFORMAT, 350};
        CHARACTER_ENCODINGS = iArr;
        ASTERISK_ENCODING = iArr[47];
    }

    @Override // com.google.zxing.oned.OneDReader
    public Result decodeRow(int i, BitArray bitArray, Map<DecodeHintType, ?> map) throws NotFoundException, ChecksumException, FormatException {
        int nextSet = bitArray.getNextSet(findAsteriskPattern(bitArray)[1]);
        int size = bitArray.getSize();
        int[] iArr = this.counters;
        Arrays.fill(iArr, 0);
        StringBuilder sb = this.decodeRowResult;
        sb.setLength(0);
        while (true) {
            recordPattern(bitArray, nextSet, iArr);
            int pattern = toPattern(iArr);
            if (pattern < 0) {
                throw NotFoundException.getNotFoundInstance();
            }
            char cPatternToChar = patternToChar(pattern);
            sb.append(cPatternToChar);
            int i2 = nextSet;
            for (int i3 : iArr) {
                i2 += i3;
            }
            int nextSet2 = bitArray.getNextSet(i2);
            if (cPatternToChar == '*') {
                sb.deleteCharAt(sb.length() - 1);
                int i4 = 0;
                for (int i5 : iArr) {
                    i4 += i5;
                }
                if (nextSet2 == size || !bitArray.get(nextSet2)) {
                    throw NotFoundException.getNotFoundInstance();
                }
                if (sb.length() < 2) {
                    throw NotFoundException.getNotFoundInstance();
                }
                checkChecksums(sb);
                sb.setLength(sb.length() - 2);
                float f = i;
                return new Result(decodeExtended(sb), null, new ResultPoint[]{new ResultPoint((r14[1] + r14[0]) / 2.0f, f), new ResultPoint(nextSet + (i4 / 2.0f), f)}, BarcodeFormat.CODE_93);
            }
            nextSet = nextSet2;
        }
    }

    private int[] findAsteriskPattern(BitArray row) throws NotFoundException {
        int width = row.getSize();
        int rowOffset = row.getNextSet(0);
        Arrays.fill(this.counters, 0);
        int[] theCounters = this.counters;
        int patternStart = rowOffset;
        boolean isWhite = false;
        int patternLength = theCounters.length;
        int counterPosition = 0;
        for (int i = rowOffset; i < width; i++) {
            if (row.get(i) != isWhite) {
                theCounters[counterPosition] = theCounters[counterPosition] + 1;
            } else {
                if (counterPosition == patternLength - 1) {
                    if (toPattern(theCounters) == ASTERISK_ENCODING) {
                        return new int[]{patternStart, i};
                    }
                    patternStart += theCounters[0] + theCounters[1];
                    System.arraycopy(theCounters, 2, theCounters, 0, counterPosition - 1);
                    theCounters[counterPosition - 1] = 0;
                    theCounters[counterPosition] = 0;
                    counterPosition--;
                } else {
                    counterPosition++;
                }
                theCounters[counterPosition] = 1;
                isWhite = isWhite ? false : true;
            }
        }
        throw NotFoundException.getNotFoundInstance();
    }

    private static int toPattern(int[] iArr) {
        int i = 0;
        for (int i2 : iArr) {
            i += i2;
        }
        int length = iArr.length;
        int i3 = 0;
        for (int i4 = 0; i4 < length; i4++) {
            int iRound = Math.round((iArr[i4] * 9.0f) / i);
            if (iRound <= 0 || iRound > 4) {
                return -1;
            }
            if ((i4 & 1) == 0) {
                for (int i5 = 0; i5 < iRound; i5++) {
                    i3 = (i3 << 1) | 1;
                }
            } else {
                i3 <<= iRound;
            }
        }
        return i3;
    }

    private static char patternToChar(int pattern) throws NotFoundException {
        for (int i = 0; i < CHARACTER_ENCODINGS.length; i++) {
            if (CHARACTER_ENCODINGS[i] == pattern) {
                return ALPHABET[i];
            }
        }
        throw NotFoundException.getNotFoundInstance();
    }

    /* JADX WARN: Can't fix incorrect switch cases order, some code will duplicate */
    private static String decodeExtended(CharSequence encoded) throws FormatException {
        int length = encoded.length();
        StringBuilder decoded = new StringBuilder(length);
        int i = 0;
        while (i < length) {
            char c = encoded.charAt(i);
            if (c >= 'a' && c <= 'd') {
                if (i >= length - 1) {
                    throw FormatException.getFormatInstance();
                }
                char next = encoded.charAt(i + 1);
                char decodedChar = 0;
                switch (c) {
                    case 'a':
                        if (next >= 'A' && next <= 'Z') {
                            decodedChar = (char) (next - '@');
                            decoded.append(decodedChar);
                            i++;
                        } else {
                            throw FormatException.getFormatInstance();
                        }
                        break;
                    case 'b':
                        if (next >= 'A' && next <= 'E') {
                            decodedChar = (char) (next - '&');
                        } else if (next >= 'F' && next <= 'J') {
                            decodedChar = (char) (next - 11);
                        } else if (next >= 'K' && next <= 'O') {
                            decodedChar = (char) (next + 16);
                        } else if (next >= 'P' && next <= 'T') {
                            decodedChar = (char) (next + '+');
                        } else if (next == 'U') {
                            decodedChar = 0;
                        } else if (next == 'V') {
                            decodedChar = '@';
                        } else if (next == 'W') {
                            decodedChar = '`';
                        } else if (next >= 'X' && next <= 'Z') {
                            decodedChar = 127;
                        } else {
                            throw FormatException.getFormatInstance();
                        }
                        decoded.append(decodedChar);
                        i++;
                        break;
                    case 'c':
                        if (next >= 'A' && next <= 'O') {
                            decodedChar = (char) (next - ' ');
                        } else if (next == 'Z') {
                            decodedChar = ':';
                        } else {
                            throw FormatException.getFormatInstance();
                        }
                        decoded.append(decodedChar);
                        i++;
                        break;
                    case 'd':
                        if (next >= 'A' && next <= 'Z') {
                            decodedChar = (char) (next + ' ');
                            decoded.append(decodedChar);
                            i++;
                        } else {
                            throw FormatException.getFormatInstance();
                        }
                        break;
                    default:
                        decoded.append(decodedChar);
                        i++;
                        break;
                }
            } else {
                decoded.append(c);
            }
            i++;
        }
        return decoded.toString();
    }

    private static void checkChecksums(CharSequence result) throws ChecksumException {
        int length = result.length();
        checkOneChecksum(result, length - 2, 20);
        checkOneChecksum(result, length - 1, 15);
    }

    private static void checkOneChecksum(CharSequence result, int checkPosition, int weightMax) throws ChecksumException {
        int weight = 1;
        int total = 0;
        for (int i = checkPosition - 1; i >= 0; i--) {
            total += ALPHABET_STRING.indexOf(result.charAt(i)) * weight;
            weight++;
            if (weight > weightMax) {
                weight = 1;
            }
        }
        int i2 = result.charAt(checkPosition);
        if (i2 != ALPHABET[total % 47]) {
            throw ChecksumException.getChecksumInstance();
        }
    }
}
