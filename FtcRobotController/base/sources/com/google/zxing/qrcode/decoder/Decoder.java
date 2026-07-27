package com.google.zxing.qrcode.decoder;

import com.google.zxing.ChecksumException;
import com.google.zxing.DecodeHintType;
import com.google.zxing.FormatException;
import com.google.zxing.common.BitMatrix;
import com.google.zxing.common.DecoderResult;
import com.google.zxing.common.reedsolomon.GenericGF;
import com.google.zxing.common.reedsolomon.ReedSolomonDecoder;
import com.google.zxing.common.reedsolomon.ReedSolomonException;
import java.util.Map;

/* JADX INFO: loaded from: classes8.dex */
public final class Decoder {
    private final ReedSolomonDecoder rsDecoder = new ReedSolomonDecoder(GenericGF.QR_CODE_FIELD_256);

    public DecoderResult decode(boolean[][] image) throws ChecksumException, FormatException {
        return decode(image, (Map<DecodeHintType, ?>) null);
    }

    public DecoderResult decode(boolean[][] image, Map<DecodeHintType, ?> hints) throws ChecksumException, FormatException {
        return decode(BitMatrix.parse(image), hints);
    }

    public DecoderResult decode(BitMatrix bits) throws ChecksumException, FormatException {
        return decode(bits, (Map<DecodeHintType, ?>) null);
    }

    public DecoderResult decode(BitMatrix bitMatrix, Map<DecodeHintType, ?> map) throws ChecksumException, FormatException {
        ChecksumException e;
        BitMatrixParser bitMatrixParser = new BitMatrixParser(bitMatrix);
        FormatException formatException = null;
        try {
            return decode(bitMatrixParser, map);
        } catch (ChecksumException e2) {
            e = e2;
            try {
                bitMatrixParser.remask();
                bitMatrixParser.setMirror(true);
                bitMatrixParser.readVersion();
                bitMatrixParser.readFormatInformation();
                bitMatrixParser.mirror();
                DecoderResult decoderResultDecode = decode(bitMatrixParser, map);
                decoderResultDecode.setOther(new QRCodeDecoderMetaData(true));
                return decoderResultDecode;
            } catch (ChecksumException | FormatException e3) {
                if (formatException != null) {
                    throw formatException;
                }
                throw e;
            }
        } catch (FormatException e4) {
            e = null;
            formatException = e4;
            bitMatrixParser.remask();
            bitMatrixParser.setMirror(true);
            bitMatrixParser.readVersion();
            bitMatrixParser.readFormatInformation();
            bitMatrixParser.mirror();
            DecoderResult decoderResultDecode2 = decode(bitMatrixParser, map);
            decoderResultDecode2.setOther(new QRCodeDecoderMetaData(true));
            return decoderResultDecode2;
        }
    }

    private DecoderResult decode(BitMatrixParser parser, Map<DecodeHintType, ?> hints) throws ChecksumException, FormatException {
        Version version = parser.readVersion();
        ErrorCorrectionLevel ecLevel = parser.readFormatInformation().getErrorCorrectionLevel();
        DataBlock[] dataBlocks = DataBlock.getDataBlocks(parser.readCodewords(), version, ecLevel);
        int totalBytes = 0;
        for (DataBlock dataBlock : dataBlocks) {
            totalBytes += dataBlock.getNumDataCodewords();
        }
        byte[] resultBytes = new byte[totalBytes];
        int resultOffset = 0;
        for (DataBlock dataBlock2 : dataBlocks) {
            byte[] codewordBytes = dataBlock2.getCodewords();
            int numDataCodewords = dataBlock2.getNumDataCodewords();
            correctErrors(codewordBytes, numDataCodewords);
            int i = 0;
            while (i < numDataCodewords) {
                resultBytes[resultOffset] = codewordBytes[i];
                i++;
                resultOffset++;
            }
        }
        return DecodedBitStreamParser.decode(resultBytes, version, ecLevel, hints);
    }

    private void correctErrors(byte[] bArr, int i) throws ChecksumException {
        int length = bArr.length;
        int[] iArr = new int[length];
        for (int i2 = 0; i2 < length; i2++) {
            iArr[i2] = bArr[i2] & 255;
        }
        try {
            this.rsDecoder.decode(iArr, bArr.length - i);
            for (int i3 = 0; i3 < i; i3++) {
                bArr[i3] = (byte) iArr[i3];
            }
        } catch (ReedSolomonException e) {
            throw ChecksumException.getChecksumInstance();
        }
    }
}
