package com.google.zxing.qrcode;

import com.google.zxing.BarcodeFormat;
import com.google.zxing.BinaryBitmap;
import com.google.zxing.ChecksumException;
import com.google.zxing.DecodeHintType;
import com.google.zxing.FormatException;
import com.google.zxing.NotFoundException;
import com.google.zxing.Reader;
import com.google.zxing.Result;
import com.google.zxing.ResultMetadataType;
import com.google.zxing.ResultPoint;
import com.google.zxing.common.BitMatrix;
import com.google.zxing.common.DecoderResult;
import com.google.zxing.common.DetectorResult;
import com.google.zxing.qrcode.decoder.Decoder;
import com.google.zxing.qrcode.decoder.QRCodeDecoderMetaData;
import com.google.zxing.qrcode.detector.Detector;
import java.util.List;
import java.util.Map;

/* JADX INFO: loaded from: classes8.dex */
public class QRCodeReader implements Reader {
    private static final ResultPoint[] NO_POINTS = new ResultPoint[0];
    private final Decoder decoder = new Decoder();

    protected final Decoder getDecoder() {
        return this.decoder;
    }

    @Override // com.google.zxing.Reader
    public Result decode(BinaryBitmap image) throws NotFoundException, ChecksumException, FormatException {
        return decode(image, null);
    }

    @Override // com.google.zxing.Reader
    public final Result decode(BinaryBitmap image, Map<DecodeHintType, ?> hints) throws NotFoundException, ChecksumException, FormatException {
        DecoderResult decoderResult;
        ResultPoint[] points;
        if (hints != null && hints.containsKey(DecodeHintType.PURE_BARCODE)) {
            BitMatrix bits = extractPureBits(image.getBlackMatrix());
            decoderResult = this.decoder.decode(bits, hints);
            points = NO_POINTS;
        } else {
            DetectorResult detectorResult = new Detector(image.getBlackMatrix()).detect(hints);
            decoderResult = this.decoder.decode(detectorResult.getBits(), hints);
            points = detectorResult.getPoints();
        }
        if (decoderResult.getOther() instanceof QRCodeDecoderMetaData) {
            ((QRCodeDecoderMetaData) decoderResult.getOther()).applyMirroredCorrection(points);
        }
        Result result = new Result(decoderResult.getText(), decoderResult.getRawBytes(), points, BarcodeFormat.QR_CODE);
        List<byte[]> byteSegments = decoderResult.getByteSegments();
        if (byteSegments != null) {
            result.putMetadata(ResultMetadataType.BYTE_SEGMENTS, byteSegments);
        }
        String ecLevel = decoderResult.getECLevel();
        if (ecLevel != null) {
            result.putMetadata(ResultMetadataType.ERROR_CORRECTION_LEVEL, ecLevel);
        }
        if (decoderResult.hasStructuredAppend()) {
            result.putMetadata(ResultMetadataType.STRUCTURED_APPEND_SEQUENCE, Integer.valueOf(decoderResult.getStructuredAppendSequenceNumber()));
            result.putMetadata(ResultMetadataType.STRUCTURED_APPEND_PARITY, Integer.valueOf(decoderResult.getStructuredAppendParity()));
        }
        return result;
    }

    @Override // com.google.zxing.Reader
    public void reset() {
    }

    private static BitMatrix extractPureBits(BitMatrix image) throws NotFoundException {
        int[] leftTopBlack = image.getTopLeftOnBit();
        int[] rightBottomBlack = image.getBottomRightOnBit();
        if (leftTopBlack == null || rightBottomBlack == null) {
            throw NotFoundException.getNotFoundInstance();
        }
        float moduleSize = moduleSize(leftTopBlack, image);
        int top = leftTopBlack[1];
        int bottom = rightBottomBlack[1];
        int left = leftTopBlack[0];
        int right = rightBottomBlack[0];
        if (left >= right || top >= bottom) {
            throw NotFoundException.getNotFoundInstance();
        }
        if (bottom - top != right - left) {
            int i = (bottom - top) + left;
            right = i;
            if (i >= image.getWidth()) {
                throw NotFoundException.getNotFoundInstance();
            }
        }
        int matrixWidth = Math.round(((right - left) + 1) / moduleSize);
        int matrixHeight = Math.round(((bottom - top) + 1) / moduleSize);
        if (matrixWidth <= 0 || matrixHeight <= 0) {
            throw NotFoundException.getNotFoundInstance();
        }
        if (matrixHeight != matrixWidth) {
            throw NotFoundException.getNotFoundInstance();
        }
        int nudge = (int) (moduleSize / 2.0f);
        int top2 = top + nudge;
        int i2 = left + nudge;
        int left2 = i2;
        int nudgedTooFarRight = (i2 + ((int) ((matrixWidth - 1) * moduleSize))) - right;
        if (nudgedTooFarRight > 0) {
            if (nudgedTooFarRight > nudge) {
                throw NotFoundException.getNotFoundInstance();
            }
            left2 -= nudgedTooFarRight;
        }
        int nudgedTooFarDown = (((int) ((matrixHeight - 1) * moduleSize)) + top2) - bottom;
        if (nudgedTooFarDown > 0) {
            if (nudgedTooFarDown > nudge) {
                throw NotFoundException.getNotFoundInstance();
            }
            top2 -= nudgedTooFarDown;
        }
        BitMatrix bits = new BitMatrix(matrixWidth, matrixHeight);
        int y = 0;
        while (y < matrixHeight) {
            int iOffset = ((int) (y * moduleSize)) + top2;
            int[] leftTopBlack2 = leftTopBlack;
            int x = 0;
            while (x < matrixWidth) {
                int[] rightBottomBlack2 = rightBottomBlack;
                if (image.get(((int) (x * moduleSize)) + left2, iOffset)) {
                    bits.set(x, y);
                }
                x++;
                rightBottomBlack = rightBottomBlack2;
            }
            y++;
            leftTopBlack = leftTopBlack2;
        }
        return bits;
    }

    private static float moduleSize(int[] leftTopBlack, BitMatrix image) throws NotFoundException {
        boolean z;
        int height = image.getHeight();
        int width = image.getWidth();
        int x = leftTopBlack[0];
        int y = leftTopBlack[1];
        boolean inBlack = true;
        int transitions = 0;
        while (x < width && y < height) {
            if (inBlack != image.get(x, y)) {
                transitions++;
                if (transitions == 5) {
                    break;
                }
                if (inBlack) {
                    z = false;
                } else {
                    z = true;
                }
                inBlack = z;
            }
            x++;
            y++;
        }
        if (x == width || y == height) {
            throw NotFoundException.getNotFoundInstance();
        }
        return (x - leftTopBlack[0]) / 7.0f;
    }
}
