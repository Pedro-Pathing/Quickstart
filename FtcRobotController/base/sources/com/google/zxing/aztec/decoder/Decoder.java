package com.google.zxing.aztec.decoder;

import com.google.zxing.FormatException;
import com.google.zxing.aztec.AztecDetectorResult;
import com.google.zxing.common.BitMatrix;
import com.google.zxing.common.DecoderResult;
import com.google.zxing.common.reedsolomon.GenericGF;
import com.google.zxing.common.reedsolomon.ReedSolomonDecoder;
import com.google.zxing.common.reedsolomon.ReedSolomonException;
import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;
import com.sun.tools.doclint.DocLint;
import java.util.Arrays;
import org.firstinspires.ftc.onbotjava.OnBotJavaFileSystemUtils;
import org.firstinspires.ftc.onbotjava.RequestConditions;
import org.slf4j.Marker;

/* JADX INFO: loaded from: classes8.dex */
public final class Decoder {
    private AztecDetectorResult ddata;
    private static final String[] UPPER_TABLE = {"CTRL_PS", " ", "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z", "CTRL_LL", "CTRL_ML", "CTRL_DL", "CTRL_BS"};
    private static final String[] LOWER_TABLE = {"CTRL_PS", " ", "a", "b", "c", "d", "e", RequestConditions.REQUEST_KEY_FILE, "g", "h", "i", "j", "k", "l", "m", "n", "o", "p", "q", "r", "s", "t", "u", "v", "w", "x", "y", "z", "CTRL_US", "CTRL_ML", "CTRL_DL", "CTRL_BS"};
    private static final String[] MIXED_TABLE = {"CTRL_PS", " ", "\u0001", "\u0002", "\u0003", "\u0004", "\u0005", "\u0006", "\u0007", "\b", "\t", "\n", "\u000b", "\f", "\r", "\u001b", "\u001c", "\u001d", "\u001e", "\u001f", "@", "\\", "^", "_", "`", "|", "~", "\u007f", "CTRL_LL", "CTRL_UL", "CTRL_PL", "CTRL_BS"};
    private static final String[] PUNCT_TABLE = {"", "\r", "\r\n", ". ", ", ", ": ", "!", "\"", "#", "$", "%", "&", "'", "(", ")", Marker.ANY_MARKER, Marker.ANY_NON_NULL_MARKER, DocLint.TAGS_SEPARATOR, "-", ".", OnBotJavaFileSystemUtils.PATH_SEPARATOR, ":", RobotConfigFileManager.FILE_LIST_COMMAND_DELIMITER, "<", "=", ">", "?", "[", "]", "{", "}", "CTRL_UL"};
    private static final String[] DIGIT_TABLE = {"CTRL_PS", " ", "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", DocLint.TAGS_SEPARATOR, ".", "CTRL_UL", "CTRL_US"};

    private enum Table {
        UPPER,
        LOWER,
        MIXED,
        DIGIT,
        PUNCT,
        BINARY
    }

    public DecoderResult decode(AztecDetectorResult detectorResult) throws FormatException {
        this.ddata = detectorResult;
        BitMatrix matrix = detectorResult.getBits();
        boolean[] rawbits = extractBits(matrix);
        boolean[] correctedBits = correctBits(rawbits);
        byte[] rawBytes = convertBoolArrayToByteArray(correctedBits);
        String result = getEncodedData(correctedBits);
        DecoderResult decoderResult = new DecoderResult(rawBytes, result, null, null);
        decoderResult.setNumBits(correctedBits.length);
        return decoderResult;
    }

    public static String highLevelDecode(boolean[] correctedBits) {
        return getEncodedData(correctedBits);
    }

    private static String getEncodedData(boolean[] zArr) {
        int length = zArr.length;
        Table table = Table.UPPER;
        Table table2 = Table.UPPER;
        StringBuilder sb = new StringBuilder(20);
        int i = 0;
        while (i < length) {
            if (table2 == Table.BINARY) {
                if (length - i < 5) {
                    break;
                }
                int code = readCode(zArr, i, 5);
                i += 5;
                if (code == 0) {
                    if (length - i < 11) {
                        break;
                    }
                    code = readCode(zArr, i, 11) + 31;
                    i += 11;
                }
                int i2 = 0;
                while (true) {
                    if (i2 >= code) {
                        break;
                    }
                    if (length - i < 8) {
                        i = length;
                        break;
                    }
                    sb.append((char) readCode(zArr, i, 8));
                    i += 8;
                    i2++;
                }
                table2 = table;
            } else {
                int i3 = table2 == Table.DIGIT ? 4 : 5;
                if (length - i < i3) {
                    break;
                }
                int code2 = readCode(zArr, i, i3);
                i += i3;
                String character = getCharacter(table2, code2);
                if (character.startsWith("CTRL_")) {
                    table = getTable(character.charAt(5));
                    if (character.charAt(6) != 'L') {
                        Table table3 = table2;
                        table2 = table;
                        table = table3;
                    } else {
                        table2 = table;
                    }
                } else {
                    sb.append(character);
                    table2 = table;
                }
            }
        }
        return sb.toString();
    }

    private static Table getTable(char t) {
        switch (t) {
            case 'B':
                return Table.BINARY;
            case 'D':
                return Table.DIGIT;
            case 'L':
                return Table.LOWER;
            case 'M':
                return Table.MIXED;
            case 'P':
                return Table.PUNCT;
            default:
                return Table.UPPER;
        }
    }

    private static String getCharacter(Table table, int code) {
        switch (table) {
            case UPPER:
                return UPPER_TABLE[code];
            case LOWER:
                return LOWER_TABLE[code];
            case MIXED:
                return MIXED_TABLE[code];
            case PUNCT:
                return PUNCT_TABLE[code];
            case DIGIT:
                return DIGIT_TABLE[code];
            default:
                throw new IllegalStateException("Bad table");
        }
    }

    private boolean[] correctBits(boolean[] zArr) throws FormatException {
        int i;
        GenericGF genericGF;
        boolean z;
        boolean z2;
        if (this.ddata.getNbLayers() <= 2) {
            genericGF = GenericGF.AZTEC_DATA_6;
            i = 6;
        } else {
            i = 8;
            if (this.ddata.getNbLayers() <= 8) {
                genericGF = GenericGF.AZTEC_DATA_8;
            } else if (this.ddata.getNbLayers() <= 22) {
                genericGF = GenericGF.AZTEC_DATA_10;
                i = 10;
            } else {
                genericGF = GenericGF.AZTEC_DATA_12;
                i = 12;
            }
        }
        int nbDatablocks = this.ddata.getNbDatablocks();
        int length = zArr.length / i;
        if (length < nbDatablocks) {
            throw FormatException.getFormatInstance();
        }
        int length2 = zArr.length % i;
        int[] iArr = new int[length];
        int i2 = 0;
        while (i2 < length) {
            iArr[i2] = readCode(zArr, length2, i);
            i2++;
            length2 += i;
        }
        try {
            new ReedSolomonDecoder(genericGF).decode(iArr, length - nbDatablocks);
            int i3 = (1 << i) - 1;
            int i4 = 0;
            for (int i5 = 0; i5 < nbDatablocks; i5++) {
                int i6 = iArr[i5];
                if (i6 == 0 || i6 == i3) {
                    throw FormatException.getFormatInstance();
                }
                if (i6 == 1 || i6 == i3 - 1) {
                    i4++;
                }
            }
            boolean[] zArr2 = new boolean[(nbDatablocks * i) - i4];
            int i7 = 0;
            for (int i8 = 0; i8 < nbDatablocks; i8++) {
                int i9 = iArr[i8];
                if (i9 == 1 || i9 == i3 - 1) {
                    int i10 = (i7 + i) - 1;
                    if (i9 <= 1) {
                        z = false;
                    } else {
                        z = true;
                    }
                    Arrays.fill(zArr2, i7, i10, z);
                    i7 += i - 1;
                } else {
                    int i11 = i - 1;
                    while (i11 >= 0) {
                        int i12 = i7 + 1;
                        if (((1 << i11) & i9) == 0) {
                            z2 = false;
                        } else {
                            z2 = true;
                        }
                        zArr2[i7] = z2;
                        i11--;
                        i7 = i12;
                    }
                }
            }
            return zArr2;
        } catch (ReedSolomonException e) {
            throw FormatException.getFormatInstance(e);
        }
    }

    private boolean[] extractBits(BitMatrix matrix) {
        boolean compact = this.ddata.isCompact();
        int layers = this.ddata.getNbLayers();
        int baseMatrixSize = (compact ? 11 : 14) + (layers << 2);
        int[] alignmentMap = new int[baseMatrixSize];
        boolean[] rawbits = new boolean[totalBitsInLayer(layers, compact)];
        int i = 2;
        if (!compact) {
            int matrixSize = baseMatrixSize + 1 + ((((baseMatrixSize / 2) - 1) / 15) * 2);
            int origCenter = baseMatrixSize / 2;
            int center = matrixSize / 2;
            for (int i2 = 0; i2 < origCenter; i2++) {
                int newOffset = (i2 / 15) + i2;
                alignmentMap[(origCenter - i2) - 1] = (center - newOffset) - 1;
                alignmentMap[origCenter + i2] = center + newOffset + 1;
            }
        } else {
            for (int i3 = 0; i3 < alignmentMap.length; i3++) {
                alignmentMap[i3] = i3;
            }
        }
        int i4 = 0;
        int rowOffset = 0;
        while (i4 < layers) {
            int rowSize = ((layers - i4) << i) + (compact ? 9 : 12);
            int low = i4 << 1;
            int high = (baseMatrixSize - 1) - low;
            int j = 0;
            while (j < rowSize) {
                int columnOffset = j << 1;
                int k = 0;
                while (k < i) {
                    rawbits[rowOffset + columnOffset + k] = matrix.get(alignmentMap[low + k], alignmentMap[low + j]);
                    rawbits[(rowSize * 2) + rowOffset + columnOffset + k] = matrix.get(alignmentMap[low + j], alignmentMap[high - k]);
                    rawbits[(rowSize * 4) + rowOffset + columnOffset + k] = matrix.get(alignmentMap[high - k], alignmentMap[high - j]);
                    rawbits[(rowSize * 6) + rowOffset + columnOffset + k] = matrix.get(alignmentMap[high - j], alignmentMap[low + k]);
                    k++;
                    i = 2;
                    compact = compact;
                }
                j++;
                i = 2;
            }
            rowOffset += rowSize << 3;
            i4++;
            i = 2;
        }
        return rawbits;
    }

    private static int readCode(boolean[] rawbits, int startIndex, int length) {
        int res = 0;
        for (int i = startIndex; i < startIndex + length; i++) {
            res <<= 1;
            if (rawbits[i]) {
                res |= 1;
            }
        }
        return res;
    }

    private static byte readByte(boolean[] rawbits, int startIndex) {
        int n = rawbits.length - startIndex;
        if (n < 8) {
            return (byte) (readCode(rawbits, startIndex, n) << (8 - n));
        }
        return (byte) readCode(rawbits, startIndex, 8);
    }

    static byte[] convertBoolArrayToByteArray(boolean[] boolArr) {
        byte[] byteArr = new byte[(boolArr.length + 7) / 8];
        for (int i = 0; i < byteArr.length; i++) {
            byteArr[i] = readByte(boolArr, i << 3);
        }
        return byteArr;
    }

    private static int totalBitsInLayer(int layers, boolean compact) {
        return ((compact ? 88 : 112) + (layers << 4)) * layers;
    }
}
