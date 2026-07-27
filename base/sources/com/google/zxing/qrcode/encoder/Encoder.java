package com.google.zxing.qrcode.encoder;

import com.google.zxing.EncodeHintType;
import com.google.zxing.WriterException;
import com.google.zxing.common.BitArray;
import com.google.zxing.common.CharacterSetECI;
import com.google.zxing.common.reedsolomon.GenericGF;
import com.google.zxing.common.reedsolomon.ReedSolomonEncoder;
import com.google.zxing.qrcode.decoder.ErrorCorrectionLevel;
import com.google.zxing.qrcode.decoder.Mode;
import com.google.zxing.qrcode.decoder.Version;
import com.sun.tools.javac.jvm.ByteCodes;
import java.io.UnsupportedEncodingException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.Map;

/* JADX INFO: loaded from: classes8.dex */
public final class Encoder {
    private static final int[] ALPHANUMERIC_TABLE = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 36, -1, -1, -1, 37, 38, -1, -1, -1, -1, 39, 40, -1, 41, 42, 43, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 44, -1, -1, -1, -1, -1, -1, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, -1, -1, -1, -1, -1};
    static final String DEFAULT_BYTE_MODE_ENCODING = "ISO-8859-1";

    private Encoder() {
    }

    private static int calculateMaskPenalty(ByteMatrix matrix) {
        return MaskUtil.applyMaskPenaltyRule1(matrix) + MaskUtil.applyMaskPenaltyRule2(matrix) + MaskUtil.applyMaskPenaltyRule3(matrix) + MaskUtil.applyMaskPenaltyRule4(matrix);
    }

    public static QRCode encode(String content, ErrorCorrectionLevel ecLevel) throws WriterException {
        return encode(content, ecLevel, null);
    }

    public static QRCode encode(String str, ErrorCorrectionLevel errorCorrectionLevel, Map<EncodeHintType, ?> map) throws WriterException {
        String string;
        Version versionRecommendVersion;
        int iChooseMaskPattern;
        CharacterSetECI characterSetECIByName;
        boolean z = map != null && map.containsKey(EncodeHintType.CHARACTER_SET);
        if (!z) {
            string = DEFAULT_BYTE_MODE_ENCODING;
        } else {
            string = map.get(EncodeHintType.CHARACTER_SET).toString();
        }
        Mode modeChooseMode = chooseMode(str, string);
        BitArray bitArray = new BitArray();
        if (modeChooseMode == Mode.BYTE && z && (characterSetECIByName = CharacterSetECI.getCharacterSetECIByName(string)) != null) {
            appendECI(characterSetECIByName, bitArray);
        }
        if ((map != null && map.containsKey(EncodeHintType.GS1_FORMAT)) && Boolean.parseBoolean(map.get(EncodeHintType.GS1_FORMAT).toString())) {
            appendModeInfo(Mode.FNC1_FIRST_POSITION, bitArray);
        }
        appendModeInfo(modeChooseMode, bitArray);
        BitArray bitArray2 = new BitArray();
        appendBytes(str, modeChooseMode, bitArray2, string);
        if (map != null && map.containsKey(EncodeHintType.QR_VERSION)) {
            versionRecommendVersion = Version.getVersionForNumber(Integer.parseInt(map.get(EncodeHintType.QR_VERSION).toString()));
            if (!willFit(calculateBitsNeeded(modeChooseMode, bitArray, bitArray2, versionRecommendVersion), versionRecommendVersion, errorCorrectionLevel)) {
                throw new WriterException("Data too big for requested version");
            }
        } else {
            versionRecommendVersion = recommendVersion(errorCorrectionLevel, modeChooseMode, bitArray, bitArray2);
        }
        BitArray bitArray3 = new BitArray();
        bitArray3.appendBitArray(bitArray);
        appendLengthInfo(modeChooseMode == Mode.BYTE ? bitArray2.getSizeInBytes() : str.length(), versionRecommendVersion, modeChooseMode, bitArray3);
        bitArray3.appendBitArray(bitArray2);
        Version.ECBlocks eCBlocksForLevel = versionRecommendVersion.getECBlocksForLevel(errorCorrectionLevel);
        int totalCodewords = versionRecommendVersion.getTotalCodewords() - eCBlocksForLevel.getTotalECCodewords();
        terminateBits(totalCodewords, bitArray3);
        BitArray bitArrayInterleaveWithECBytes = interleaveWithECBytes(bitArray3, versionRecommendVersion.getTotalCodewords(), totalCodewords, eCBlocksForLevel.getNumBlocks());
        QRCode qRCode = new QRCode();
        qRCode.setECLevel(errorCorrectionLevel);
        qRCode.setMode(modeChooseMode);
        qRCode.setVersion(versionRecommendVersion);
        int dimensionForVersion = versionRecommendVersion.getDimensionForVersion();
        ByteMatrix byteMatrix = new ByteMatrix(dimensionForVersion, dimensionForVersion);
        if (map != null && map.containsKey(EncodeHintType.QR_MASK_PATTERN)) {
            iChooseMaskPattern = Integer.parseInt(map.get(EncodeHintType.QR_MASK_PATTERN).toString());
            if (!QRCode.isValidMaskPattern(iChooseMaskPattern)) {
                iChooseMaskPattern = -1;
            }
        } else {
            iChooseMaskPattern = -1;
        }
        if (iChooseMaskPattern == -1) {
            iChooseMaskPattern = chooseMaskPattern(bitArrayInterleaveWithECBytes, errorCorrectionLevel, versionRecommendVersion, byteMatrix);
        }
        qRCode.setMaskPattern(iChooseMaskPattern);
        MatrixUtil.buildMatrix(bitArrayInterleaveWithECBytes, errorCorrectionLevel, versionRecommendVersion, iChooseMaskPattern, byteMatrix);
        qRCode.setMatrix(byteMatrix);
        return qRCode;
    }

    private static Version recommendVersion(ErrorCorrectionLevel ecLevel, Mode mode, BitArray headerBits, BitArray dataBits) throws WriterException {
        Version provisionalVersion = chooseVersion(calculateBitsNeeded(mode, headerBits, dataBits, Version.getVersionForNumber(1)), ecLevel);
        return chooseVersion(calculateBitsNeeded(mode, headerBits, dataBits, provisionalVersion), ecLevel);
    }

    private static int calculateBitsNeeded(Mode mode, BitArray headerBits, BitArray dataBits, Version version) {
        return headerBits.getSize() + mode.getCharacterCountBits(version) + dataBits.getSize();
    }

    static int getAlphanumericCode(int code) {
        if (code < ALPHANUMERIC_TABLE.length) {
            return ALPHANUMERIC_TABLE[code];
        }
        return -1;
    }

    public static Mode chooseMode(String content) {
        return chooseMode(content, null);
    }

    private static Mode chooseMode(String content, String encoding) {
        if ("Shift_JIS".equals(encoding) && isOnlyDoubleByteKanji(content)) {
            return Mode.KANJI;
        }
        boolean hasNumeric = false;
        boolean hasAlphanumeric = false;
        for (int i = 0; i < content.length(); i++) {
            char c = content.charAt(i);
            if (c >= '0' && c <= '9') {
                hasNumeric = true;
            } else if (getAlphanumericCode(c) != -1) {
                hasAlphanumeric = true;
            } else {
                return Mode.BYTE;
            }
        }
        if (hasAlphanumeric) {
            return Mode.ALPHANUMERIC;
        }
        if (hasNumeric) {
            return Mode.NUMERIC;
        }
        return Mode.BYTE;
    }

    private static boolean isOnlyDoubleByteKanji(String str) {
        try {
            byte[] bytes = str.getBytes("Shift_JIS");
            int length = bytes.length;
            if (length % 2 != 0) {
                return false;
            }
            for (int i = 0; i < length; i += 2) {
                int i2 = bytes[i] & 255;
                if ((i2 < 129 || i2 > 159) && (i2 < 224 || i2 > 235)) {
                    return false;
                }
            }
            return true;
        } catch (UnsupportedEncodingException e) {
            return false;
        }
    }

    private static int chooseMaskPattern(BitArray bits, ErrorCorrectionLevel ecLevel, Version version, ByteMatrix matrix) throws WriterException {
        int minPenalty = Integer.MAX_VALUE;
        int bestMaskPattern = -1;
        for (int maskPattern = 0; maskPattern < 8; maskPattern++) {
            MatrixUtil.buildMatrix(bits, ecLevel, version, maskPattern, matrix);
            int penalty = calculateMaskPenalty(matrix);
            if (penalty < minPenalty) {
                minPenalty = penalty;
                bestMaskPattern = maskPattern;
            }
        }
        return bestMaskPattern;
    }

    private static Version chooseVersion(int numInputBits, ErrorCorrectionLevel ecLevel) throws WriterException {
        for (int versionNum = 1; versionNum <= 40; versionNum++) {
            Version version = Version.getVersionForNumber(versionNum);
            if (willFit(numInputBits, version, ecLevel)) {
                return version;
            }
        }
        throw new WriterException("Data too big");
    }

    private static boolean willFit(int numInputBits, Version version, ErrorCorrectionLevel ecLevel) {
        int numBytes = version.getTotalCodewords();
        int numEcBytes = version.getECBlocksForLevel(ecLevel).getTotalECCodewords();
        int numDataBytes = numBytes - numEcBytes;
        int totalInputBytes = (numInputBits + 7) / 8;
        return numDataBytes >= totalInputBytes;
    }

    static void terminateBits(int i, BitArray bitArray) throws WriterException {
        int i2 = i << 3;
        if (bitArray.getSize() > i2) {
            throw new WriterException("data bits cannot fit in the QR Code" + bitArray.getSize() + " > " + i2);
        }
        for (int i3 = 0; i3 < 4 && bitArray.getSize() < i2; i3++) {
            bitArray.appendBit(false);
        }
        int size = bitArray.getSize() & 7;
        if (size > 0) {
            while (size < 8) {
                bitArray.appendBit(false);
                size++;
            }
        }
        int sizeInBytes = i - bitArray.getSizeInBytes();
        for (int i4 = 0; i4 < sizeInBytes; i4++) {
            bitArray.appendBits((i4 & 1) == 0 ? 236 : 17, 8);
        }
        if (bitArray.getSize() != i2) {
            throw new WriterException("Bits size does not equal capacity");
        }
    }

    static void getNumDataBytesAndNumECBytesForBlockID(int numTotalBytes, int numDataBytes, int numRSBlocks, int blockID, int[] numDataBytesInBlock, int[] numECBytesInBlock) throws WriterException {
        if (blockID >= numRSBlocks) {
            throw new WriterException("Block ID too large");
        }
        int numRsBlocksInGroup2 = numTotalBytes % numRSBlocks;
        int numRsBlocksInGroup1 = numRSBlocks - numRsBlocksInGroup2;
        int numTotalBytesInGroup1 = numTotalBytes / numRSBlocks;
        int numTotalBytesInGroup2 = numTotalBytesInGroup1 + 1;
        int numDataBytesInGroup1 = numDataBytes / numRSBlocks;
        int numDataBytesInGroup2 = numDataBytesInGroup1 + 1;
        int numEcBytesInGroup1 = numTotalBytesInGroup1 - numDataBytesInGroup1;
        int numEcBytesInGroup2 = numTotalBytesInGroup2 - numDataBytesInGroup2;
        if (numEcBytesInGroup1 != numEcBytesInGroup2) {
            throw new WriterException("EC bytes mismatch");
        }
        if (numRSBlocks != numRsBlocksInGroup1 + numRsBlocksInGroup2) {
            throw new WriterException("RS blocks mismatch");
        }
        if (numTotalBytes != ((numDataBytesInGroup1 + numEcBytesInGroup1) * numRsBlocksInGroup1) + ((numDataBytesInGroup2 + numEcBytesInGroup2) * numRsBlocksInGroup2)) {
            throw new WriterException("Total bytes mismatch");
        }
        if (blockID < numRsBlocksInGroup1) {
            numDataBytesInBlock[0] = numDataBytesInGroup1;
            numECBytesInBlock[0] = numEcBytesInGroup1;
        } else {
            numDataBytesInBlock[0] = numDataBytesInGroup2;
            numECBytesInBlock[0] = numEcBytesInGroup2;
        }
    }

    static BitArray interleaveWithECBytes(BitArray bitArray, int i, int i2, int i3) throws WriterException {
        if (bitArray.getSizeInBytes() != i2) {
            throw new WriterException("Number of bits and data bytes does not match");
        }
        ArrayList arrayList = new ArrayList(i3);
        int i4 = 0;
        int iMax = 0;
        int iMax2 = 0;
        for (int i5 = 0; i5 < i3; i5++) {
            int[] iArr = new int[1];
            int[] iArr2 = new int[1];
            getNumDataBytesAndNumECBytesForBlockID(i, i2, i3, i5, iArr, iArr2);
            int i6 = iArr[0];
            byte[] bArr = new byte[i6];
            bitArray.toBytes(i4 << 3, bArr, 0, i6);
            byte[] bArrGenerateECBytes = generateECBytes(bArr, iArr2[0]);
            arrayList.add(new BlockPair(bArr, bArrGenerateECBytes));
            iMax = Math.max(iMax, i6);
            iMax2 = Math.max(iMax2, bArrGenerateECBytes.length);
            i4 += iArr[0];
        }
        if (i2 != i4) {
            throw new WriterException("Data bytes does not match offset");
        }
        BitArray bitArray2 = new BitArray();
        for (int i7 = 0; i7 < iMax; i7++) {
            Iterator it = arrayList.iterator();
            while (it.hasNext()) {
                byte[] dataBytes = ((BlockPair) it.next()).getDataBytes();
                if (i7 < dataBytes.length) {
                    bitArray2.appendBits(dataBytes[i7], 8);
                }
            }
        }
        for (int i8 = 0; i8 < iMax2; i8++) {
            Iterator it2 = arrayList.iterator();
            while (it2.hasNext()) {
                byte[] errorCorrectionBytes = ((BlockPair) it2.next()).getErrorCorrectionBytes();
                if (i8 < errorCorrectionBytes.length) {
                    bitArray2.appendBits(errorCorrectionBytes[i8], 8);
                }
            }
        }
        if (i != bitArray2.getSizeInBytes()) {
            throw new WriterException("Interleaving error: " + i + " and " + bitArray2.getSizeInBytes() + " differ.");
        }
        return bitArray2;
    }

    static byte[] generateECBytes(byte[] dataBytes, int numEcBytesInBlock) {
        int numDataBytes = dataBytes.length;
        int[] toEncode = new int[numDataBytes + numEcBytesInBlock];
        for (int i = 0; i < numDataBytes; i++) {
            toEncode[i] = dataBytes[i] & 255;
        }
        new ReedSolomonEncoder(GenericGF.QR_CODE_FIELD_256).encode(toEncode, numEcBytesInBlock);
        byte[] ecBytes = new byte[numEcBytesInBlock];
        for (int i2 = 0; i2 < numEcBytesInBlock; i2++) {
            ecBytes[i2] = (byte) toEncode[numDataBytes + i2];
        }
        return ecBytes;
    }

    static void appendModeInfo(Mode mode, BitArray bits) {
        bits.appendBits(mode.getBits(), 4);
    }

    static void appendLengthInfo(int numLetters, Version version, Mode mode, BitArray bits) throws WriterException {
        int numBits = mode.getCharacterCountBits(version);
        if (numLetters >= (1 << numBits)) {
            throw new WriterException(numLetters + " is bigger than " + ((1 << numBits) - 1));
        }
        bits.appendBits(numLetters, numBits);
    }

    static void appendBytes(String content, Mode mode, BitArray bits, String encoding) throws WriterException {
        switch (mode) {
            case NUMERIC:
                appendNumericBytes(content, bits);
                return;
            case ALPHANUMERIC:
                appendAlphanumericBytes(content, bits);
                return;
            case BYTE:
                append8BitBytes(content, bits, encoding);
                return;
            case KANJI:
                appendKanjiBytes(content, bits);
                return;
            default:
                throw new WriterException("Invalid mode: ".concat(String.valueOf(mode)));
        }
    }

    static void appendNumericBytes(CharSequence content, BitArray bits) {
        int length = content.length();
        int i = 0;
        while (i < length) {
            int num1 = content.charAt(i) - '0';
            if (i + 2 < length) {
                int num2 = content.charAt(i + 1) - '0';
                int num3 = content.charAt(i + 2) - '0';
                bits.appendBits((num1 * 100) + (num2 * 10) + num3, 10);
                i += 3;
            } else if (i + 1 < length) {
                int num22 = content.charAt(i + 1) - '0';
                bits.appendBits((num1 * 10) + num22, 7);
                i += 2;
            } else {
                bits.appendBits(num1, 4);
                i++;
            }
        }
    }

    static void appendAlphanumericBytes(CharSequence content, BitArray bits) throws WriterException {
        int length = content.length();
        int i = 0;
        while (i < length) {
            int code1 = getAlphanumericCode(content.charAt(i));
            if (code1 == -1) {
                throw new WriterException();
            }
            if (i + 1 >= length) {
                bits.appendBits(code1, 6);
                i++;
            } else {
                int code2 = getAlphanumericCode(content.charAt(i + 1));
                if (code2 != -1) {
                    bits.appendBits((code1 * 45) + code2, 11);
                    i += 2;
                } else {
                    throw new WriterException();
                }
            }
        }
    }

    static void append8BitBytes(String content, BitArray bits, String encoding) throws WriterException {
        try {
            byte[] bytes = content.getBytes(encoding);
            for (byte b : bytes) {
                bits.appendBits(b, 8);
            }
        } catch (UnsupportedEncodingException uee) {
            throw new WriterException(uee);
        }
    }

    static void appendKanjiBytes(String content, BitArray bits) throws WriterException {
        try {
            byte[] bytes = content.getBytes("Shift_JIS");
            if (bytes.length % 2 != 0) {
                throw new WriterException("Kanji byte size not even");
            }
            int maxI = bytes.length - 1;
            for (int i = 0; i < maxI; i += 2) {
                int byte1 = bytes[i] & 255;
                int byte2 = bytes[i + 1] & 255;
                int code = (byte1 << 8) | byte2;
                int subtracted = -1;
                if (code >= 33088 && code <= 40956) {
                    subtracted = code - 33088;
                } else if (code >= 57408 && code <= 60351) {
                    subtracted = code - 49472;
                }
                if (subtracted == -1) {
                    throw new WriterException("Invalid byte sequence");
                }
                int encoded = ((subtracted >> 8) * ByteCodes.checkcast) + (subtracted & 255);
                bits.appendBits(encoded, 13);
            }
        } catch (UnsupportedEncodingException uee) {
            throw new WriterException(uee);
        }
    }

    private static void appendECI(CharacterSetECI eci, BitArray bits) {
        bits.appendBits(Mode.ECI.getBits(), 4);
        bits.appendBits(eci.getValue(), 8);
    }
}
