package com.google.zxing.pdf417.decoder;

import com.google.zxing.FormatException;
import com.google.zxing.common.CharacterSetECI;
import com.google.zxing.common.DecoderResult;
import com.google.zxing.pdf417.PDF417ResultMetadata;
import java.io.ByteArrayOutputStream;
import java.math.BigInteger;
import java.nio.charset.Charset;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;

/* JADX INFO: loaded from: classes8.dex */
final class DecodedBitStreamParser {
    private static final int AL = 28;
    private static final int AS = 27;
    private static final int BEGIN_MACRO_PDF417_CONTROL_BLOCK = 928;
    private static final int BEGIN_MACRO_PDF417_OPTIONAL_FIELD = 923;
    private static final int BYTE_COMPACTION_MODE_LATCH = 901;
    private static final int BYTE_COMPACTION_MODE_LATCH_6 = 924;
    private static final int ECI_CHARSET = 927;
    private static final int ECI_GENERAL_PURPOSE = 926;
    private static final int ECI_USER_DEFINED = 925;
    private static final BigInteger[] EXP900;
    private static final int LL = 27;
    private static final int MACRO_PDF417_OPTIONAL_FIELD_ADDRESSEE = 4;
    private static final int MACRO_PDF417_OPTIONAL_FIELD_CHECKSUM = 6;
    private static final int MACRO_PDF417_OPTIONAL_FIELD_FILE_NAME = 0;
    private static final int MACRO_PDF417_OPTIONAL_FIELD_FILE_SIZE = 5;
    private static final int MACRO_PDF417_OPTIONAL_FIELD_SEGMENT_COUNT = 1;
    private static final int MACRO_PDF417_OPTIONAL_FIELD_SENDER = 3;
    private static final int MACRO_PDF417_OPTIONAL_FIELD_TIME_STAMP = 2;
    private static final int MACRO_PDF417_TERMINATOR = 922;
    private static final int MAX_NUMERIC_CODEWORDS = 15;
    private static final int ML = 28;
    private static final int MODE_SHIFT_TO_BYTE_COMPACTION_MODE = 913;
    private static final int NUMBER_OF_SEQUENCE_CODEWORDS = 2;
    private static final int NUMERIC_COMPACTION_MODE_LATCH = 902;
    private static final int PAL = 29;
    private static final int PL = 25;
    private static final int PS = 29;
    private static final int TEXT_COMPACTION_MODE_LATCH = 900;
    private static final char[] PUNCT_CHARS = ";<>@[\\]_`~!\r\t,:\n-.$/\"|*()?{}'".toCharArray();
    private static final char[] MIXED_CHARS = "0123456789&\r\t,:#-.$/+%*=^".toCharArray();

    private enum Mode {
        ALPHA,
        LOWER,
        MIXED,
        PUNCT,
        ALPHA_SHIFT,
        PUNCT_SHIFT
    }

    static {
        BigInteger[] bigIntegerArr = new BigInteger[16];
        EXP900 = bigIntegerArr;
        bigIntegerArr[0] = BigInteger.ONE;
        BigInteger nineHundred = BigInteger.valueOf(900L);
        EXP900[1] = nineHundred;
        for (int i = 2; i < EXP900.length; i++) {
            BigInteger[] bigIntegerArr2 = EXP900;
            bigIntegerArr2[i] = bigIntegerArr2[i - 1].multiply(nineHundred);
        }
    }

    private DecodedBitStreamParser() {
    }

    static DecoderResult decode(int[] iArr, String str) throws FormatException {
        int iTextCompaction;
        StringBuilder sb = new StringBuilder(iArr.length << 1);
        Charset charsetForName = StandardCharsets.ISO_8859_1;
        int i = iArr[1];
        PDF417ResultMetadata pDF417ResultMetadata = new PDF417ResultMetadata();
        int i2 = 2;
        while (i2 < iArr[0]) {
            switch (i) {
                case 900:
                    iTextCompaction = textCompaction(iArr, i2, sb);
                    break;
                case BYTE_COMPACTION_MODE_LATCH /* 901 */:
                case BYTE_COMPACTION_MODE_LATCH_6 /* 924 */:
                    iTextCompaction = byteCompaction(i, iArr, charsetForName, i2, sb);
                    break;
                case NUMERIC_COMPACTION_MODE_LATCH /* 902 */:
                    iTextCompaction = numericCompaction(iArr, i2, sb);
                    break;
                case MODE_SHIFT_TO_BYTE_COMPACTION_MODE /* 913 */:
                    iTextCompaction = i2 + 1;
                    sb.append((char) iArr[i2]);
                    break;
                case MACRO_PDF417_TERMINATOR /* 922 */:
                case BEGIN_MACRO_PDF417_OPTIONAL_FIELD /* 923 */:
                    throw FormatException.getFormatInstance();
                case ECI_USER_DEFINED /* 925 */:
                    iTextCompaction = i2 + 1;
                    break;
                case ECI_GENERAL_PURPOSE /* 926 */:
                    iTextCompaction = i2 + 2;
                    break;
                case ECI_CHARSET /* 927 */:
                    iTextCompaction = i2 + 1;
                    charsetForName = Charset.forName(CharacterSetECI.getCharacterSetECIByValue(iArr[i2]).name());
                    break;
                case 928:
                    iTextCompaction = decodeMacroBlock(iArr, i2, pDF417ResultMetadata);
                    break;
                default:
                    iTextCompaction = textCompaction(iArr, i2 - 1, sb);
                    break;
            }
            if (iTextCompaction < iArr.length) {
                i2 = iTextCompaction + 1;
                i = iArr[iTextCompaction];
            } else {
                throw FormatException.getFormatInstance();
            }
        }
        if (sb.length() == 0) {
            throw FormatException.getFormatInstance();
        }
        DecoderResult decoderResult = new DecoderResult(null, sb.toString(), null, str);
        decoderResult.setOther(pDF417ResultMetadata);
        return decoderResult;
    }

    static int decodeMacroBlock(int[] codewords, int codeIndex, PDF417ResultMetadata resultMetadata) throws FormatException {
        if (codeIndex + 2 > codewords[0]) {
            throw FormatException.getFormatInstance();
        }
        int[] segmentIndexArray = new int[2];
        int i = 0;
        while (i < 2) {
            segmentIndexArray[i] = codewords[codeIndex];
            i++;
            codeIndex++;
        }
        resultMetadata.setSegmentIndex(Integer.parseInt(decodeBase900toBase10(segmentIndexArray, 2)));
        StringBuilder fileId = new StringBuilder();
        int codeIndex2 = textCompaction(codewords, codeIndex, fileId);
        resultMetadata.setFileId(fileId.toString());
        int optionalFieldsStart = -1;
        if (codewords[codeIndex2] == BEGIN_MACRO_PDF417_OPTIONAL_FIELD) {
            optionalFieldsStart = codeIndex2 + 1;
        }
        while (codeIndex2 < codewords[0]) {
            switch (codewords[codeIndex2]) {
                case MACRO_PDF417_TERMINATOR /* 922 */:
                    codeIndex2++;
                    resultMetadata.setLastSegment(true);
                    break;
                case BEGIN_MACRO_PDF417_OPTIONAL_FIELD /* 923 */:
                    int codeIndex3 = codeIndex2 + 1;
                    switch (codewords[codeIndex3]) {
                        case 0:
                            StringBuilder fileName = new StringBuilder();
                            codeIndex2 = textCompaction(codewords, codeIndex3 + 1, fileName);
                            resultMetadata.setFileName(fileName.toString());
                            break;
                        case 1:
                            StringBuilder segmentCount = new StringBuilder();
                            codeIndex2 = numericCompaction(codewords, codeIndex3 + 1, segmentCount);
                            resultMetadata.setSegmentCount(Integer.parseInt(segmentCount.toString()));
                            break;
                        case 2:
                            StringBuilder timestamp = new StringBuilder();
                            codeIndex2 = numericCompaction(codewords, codeIndex3 + 1, timestamp);
                            resultMetadata.setTimestamp(Long.parseLong(timestamp.toString()));
                            break;
                        case 3:
                            StringBuilder sender = new StringBuilder();
                            codeIndex2 = textCompaction(codewords, codeIndex3 + 1, sender);
                            resultMetadata.setSender(sender.toString());
                            break;
                        case 4:
                            StringBuilder addressee = new StringBuilder();
                            codeIndex2 = textCompaction(codewords, codeIndex3 + 1, addressee);
                            resultMetadata.setAddressee(addressee.toString());
                            break;
                        case 5:
                            StringBuilder fileSize = new StringBuilder();
                            codeIndex2 = numericCompaction(codewords, codeIndex3 + 1, fileSize);
                            resultMetadata.setFileSize(Long.parseLong(fileSize.toString()));
                            break;
                        case 6:
                            StringBuilder checksum = new StringBuilder();
                            codeIndex2 = numericCompaction(codewords, codeIndex3 + 1, checksum);
                            resultMetadata.setChecksum(Integer.parseInt(checksum.toString()));
                            break;
                        default:
                            throw FormatException.getFormatInstance();
                    }
                    break;
                default:
                    throw FormatException.getFormatInstance();
            }
        }
        if (optionalFieldsStart != -1) {
            int optionalFieldsLength = codeIndex2 - optionalFieldsStart;
            if (resultMetadata.isLastSegment()) {
                optionalFieldsLength--;
            }
            resultMetadata.setOptionalData(Arrays.copyOfRange(codewords, optionalFieldsStart, optionalFieldsStart + optionalFieldsLength));
        }
        return codeIndex2;
    }

    private static int textCompaction(int[] codewords, int codeIndex, StringBuilder result) {
        int[] textCompactionData = new int[(codewords[0] - codeIndex) << 1];
        int[] byteCompactionData = new int[(codewords[0] - codeIndex) << 1];
        int index = 0;
        boolean end = false;
        while (codeIndex < codewords[0] && !end) {
            int codeIndex2 = codeIndex + 1;
            int code = codewords[codeIndex];
            if (code < 900) {
                textCompactionData[index] = code / 30;
                textCompactionData[index + 1] = code % 30;
                index += 2;
                codeIndex = codeIndex2;
            } else {
                switch (code) {
                    case 900:
                        textCompactionData[index] = 900;
                        index++;
                        codeIndex = codeIndex2;
                        break;
                    case BYTE_COMPACTION_MODE_LATCH /* 901 */:
                    case NUMERIC_COMPACTION_MODE_LATCH /* 902 */:
                    case MACRO_PDF417_TERMINATOR /* 922 */:
                    case BEGIN_MACRO_PDF417_OPTIONAL_FIELD /* 923 */:
                    case BYTE_COMPACTION_MODE_LATCH_6 /* 924 */:
                    case 928:
                        codeIndex = codeIndex2 - 1;
                        end = true;
                        break;
                    case MODE_SHIFT_TO_BYTE_COMPACTION_MODE /* 913 */:
                        textCompactionData[index] = MODE_SHIFT_TO_BYTE_COMPACTION_MODE;
                        codeIndex = codeIndex2 + 1;
                        byteCompactionData[index] = codewords[codeIndex2];
                        index++;
                        break;
                    default:
                        codeIndex = codeIndex2;
                        break;
                }
            }
        }
        decodeTextCompaction(textCompactionData, byteCompactionData, index, result);
        return codeIndex;
    }

    private static void decodeTextCompaction(int[] textCompactionData, int[] byteCompactionData, int length, StringBuilder result) {
        Mode subMode = Mode.ALPHA;
        Mode priorToShiftMode = Mode.ALPHA;
        for (int i = 0; i < length; i++) {
            int subModeCh = textCompactionData[i];
            char ch = 0;
            switch (subMode) {
                case ALPHA:
                    if (subModeCh < 26) {
                        ch = (char) (subModeCh + 65);
                        break;
                    } else {
                        switch (subModeCh) {
                            case 26:
                                ch = ' ';
                                break;
                            case 27:
                                subMode = Mode.LOWER;
                                break;
                            case 28:
                                subMode = Mode.MIXED;
                                break;
                            case 29:
                                priorToShiftMode = subMode;
                                subMode = Mode.PUNCT_SHIFT;
                                break;
                            case 900:
                                subMode = Mode.ALPHA;
                                break;
                            case MODE_SHIFT_TO_BYTE_COMPACTION_MODE /* 913 */:
                                result.append((char) byteCompactionData[i]);
                                break;
                        }
                    }
                    break;
                case LOWER:
                    if (subModeCh < 26) {
                        ch = (char) (subModeCh + 97);
                        break;
                    } else {
                        switch (subModeCh) {
                            case 26:
                                ch = ' ';
                                break;
                            case 27:
                                priorToShiftMode = subMode;
                                subMode = Mode.ALPHA_SHIFT;
                                break;
                            case 28:
                                subMode = Mode.MIXED;
                                break;
                            case 29:
                                priorToShiftMode = subMode;
                                subMode = Mode.PUNCT_SHIFT;
                                break;
                            case 900:
                                subMode = Mode.ALPHA;
                                break;
                            case MODE_SHIFT_TO_BYTE_COMPACTION_MODE /* 913 */:
                                result.append((char) byteCompactionData[i]);
                                break;
                        }
                    }
                    break;
                case MIXED:
                    if (subModeCh < 25) {
                        ch = MIXED_CHARS[subModeCh];
                        break;
                    } else {
                        switch (subModeCh) {
                            case 25:
                                subMode = Mode.PUNCT;
                                break;
                            case 26:
                                ch = ' ';
                                break;
                            case 27:
                                subMode = Mode.LOWER;
                                break;
                            case 28:
                            case 900:
                                subMode = Mode.ALPHA;
                                break;
                            case 29:
                                priorToShiftMode = subMode;
                                subMode = Mode.PUNCT_SHIFT;
                                break;
                            case MODE_SHIFT_TO_BYTE_COMPACTION_MODE /* 913 */:
                                result.append((char) byteCompactionData[i]);
                                break;
                        }
                    }
                    break;
                case PUNCT:
                    if (subModeCh < 29) {
                        ch = PUNCT_CHARS[subModeCh];
                        break;
                    } else {
                        switch (subModeCh) {
                            case 29:
                            case 900:
                                subMode = Mode.ALPHA;
                                break;
                            case MODE_SHIFT_TO_BYTE_COMPACTION_MODE /* 913 */:
                                result.append((char) byteCompactionData[i]);
                                break;
                        }
                    }
                    break;
                case ALPHA_SHIFT:
                    subMode = priorToShiftMode;
                    if (subModeCh < 26) {
                        ch = (char) (subModeCh + 65);
                        break;
                    } else {
                        switch (subModeCh) {
                            case 26:
                                ch = ' ';
                                break;
                            case 900:
                                subMode = Mode.ALPHA;
                                break;
                        }
                    }
                    break;
                case PUNCT_SHIFT:
                    subMode = priorToShiftMode;
                    if (subModeCh < 29) {
                        ch = PUNCT_CHARS[subModeCh];
                        break;
                    } else {
                        switch (subModeCh) {
                            case 29:
                            case 900:
                                subMode = Mode.ALPHA;
                                break;
                            case MODE_SHIFT_TO_BYTE_COMPACTION_MODE /* 913 */:
                                result.append((char) byteCompactionData[i]);
                                break;
                        }
                    }
                    break;
            }
            if (ch != 0) {
                result.append(ch);
            }
        }
    }

    private static int byteCompaction(int i, int[] iArr, Charset charset, int i2, StringBuilder sb) {
        int i3;
        ByteArrayOutputStream byteArrayOutputStream = new ByteArrayOutputStream();
        long j = 900;
        switch (i) {
            case BYTE_COMPACTION_MODE_LATCH /* 901 */:
                int[] iArr2 = new int[6];
                i3 = i2 + 1;
                int i4 = iArr[i2];
                boolean z = false;
                int i5 = 0;
                long j2 = 0;
                while (i3 < iArr[0] && !z) {
                    int i6 = i5 + 1;
                    iArr2[i5] = i4;
                    j2 = (j2 * 900) + ((long) i4);
                    int i7 = i3 + 1;
                    int i8 = iArr[i3];
                    switch (i8) {
                        case 900:
                        case BYTE_COMPACTION_MODE_LATCH /* 901 */:
                        case NUMERIC_COMPACTION_MODE_LATCH /* 902 */:
                        case MACRO_PDF417_TERMINATOR /* 922 */:
                        case BEGIN_MACRO_PDF417_OPTIONAL_FIELD /* 923 */:
                        case BYTE_COMPACTION_MODE_LATCH_6 /* 924 */:
                        case 928:
                            int i9 = i7 - 1;
                            z = true;
                            i5 = i6;
                            i4 = i8;
                            i3 = i9;
                            break;
                        default:
                            if (i6 % 5 != 0 || i6 <= 0) {
                                i5 = i6;
                                i4 = i8;
                                i3 = i7;
                            } else {
                                for (int i10 = 0; i10 < 6; i10++) {
                                    byteArrayOutputStream.write((byte) (j2 >> ((5 - i10) * 8)));
                                }
                                i5 = 0;
                                j2 = 0;
                                i4 = i8;
                                i3 = i7;
                            }
                            break;
                    }
                }
                if (i3 == iArr[0] && i4 < 900) {
                    iArr2[i5] = i4;
                    i5++;
                }
                for (int i11 = 0; i11 < i5; i11++) {
                    byteArrayOutputStream.write((byte) iArr2[i11]);
                }
                break;
            case BYTE_COMPACTION_MODE_LATCH_6 /* 924 */:
                int i12 = i2;
                boolean z2 = false;
                int i13 = 0;
                long j3 = 0;
                while (i12 < iArr[0] && !z2) {
                    int i14 = i12 + 1;
                    int i15 = iArr[i12];
                    if (i15 < 900) {
                        i13++;
                        j3 = (j3 * j) + ((long) i15);
                        i12 = i14;
                    } else {
                        switch (i15) {
                            case 900:
                            case BYTE_COMPACTION_MODE_LATCH /* 901 */:
                            case NUMERIC_COMPACTION_MODE_LATCH /* 902 */:
                            case MACRO_PDF417_TERMINATOR /* 922 */:
                            case BEGIN_MACRO_PDF417_OPTIONAL_FIELD /* 923 */:
                            case BYTE_COMPACTION_MODE_LATCH_6 /* 924 */:
                            case 928:
                                z2 = true;
                                i12 = i14 - 1;
                                break;
                            default:
                                i12 = i14;
                                break;
                        }
                    }
                    if (i13 % 5 == 0 && i13 > 0) {
                        for (int i16 = 0; i16 < 6; i16++) {
                            byteArrayOutputStream.write((byte) (j3 >> ((5 - i16) * 8)));
                        }
                        i13 = 0;
                        j3 = 0;
                    }
                    j = 900;
                }
                i3 = i12;
                break;
            default:
                i3 = i2;
                break;
        }
        sb.append(new String(byteArrayOutputStream.toByteArray(), charset));
        return i3;
    }

    private static int numericCompaction(int[] codewords, int code, StringBuilder result) throws FormatException {
        int count = 0;
        boolean end = false;
        int[] numericCodewords = new int[15];
        while (code < codewords[0] && !end) {
            int codeIndex = code + 1;
            int code2 = codewords[code];
            if (codeIndex == codewords[0]) {
                end = true;
            }
            if (code2 < 900) {
                numericCodewords[count] = code2;
                count++;
            } else {
                switch (code2) {
                    case 900:
                    case BYTE_COMPACTION_MODE_LATCH /* 901 */:
                    case MACRO_PDF417_TERMINATOR /* 922 */:
                    case BEGIN_MACRO_PDF417_OPTIONAL_FIELD /* 923 */:
                    case BYTE_COMPACTION_MODE_LATCH_6 /* 924 */:
                    case 928:
                        codeIndex--;
                        end = true;
                        break;
                }
            }
            if ((count % 15 == 0 || code2 == NUMERIC_COMPACTION_MODE_LATCH || end) && count > 0) {
                result.append(decodeBase900toBase10(numericCodewords, count));
                count = 0;
            }
            code = codeIndex;
        }
        return code;
    }

    private static String decodeBase900toBase10(int[] codewords, int count) throws FormatException {
        BigInteger result = BigInteger.ZERO;
        for (int i = 0; i < count; i++) {
            result = result.add(EXP900[(count - i) - 1].multiply(BigInteger.valueOf(codewords[i])));
        }
        String resultString = result.toString();
        if (resultString.charAt(0) != '1') {
            throw FormatException.getFormatInstance();
        }
        return resultString.substring(1);
    }
}
