package com.google.zxing.qrcode;

import com.google.zxing.BarcodeFormat;
import com.google.zxing.EncodeHintType;
import com.google.zxing.Writer;
import com.google.zxing.WriterException;
import com.google.zxing.common.BitMatrix;
import com.google.zxing.qrcode.decoder.ErrorCorrectionLevel;
import com.google.zxing.qrcode.encoder.ByteMatrix;
import com.google.zxing.qrcode.encoder.Encoder;
import com.google.zxing.qrcode.encoder.QRCode;
import java.util.Map;

/* JADX INFO: loaded from: classes8.dex */
public final class QRCodeWriter implements Writer {
    private static final int QUIET_ZONE_SIZE = 4;

    @Override // com.google.zxing.Writer
    public BitMatrix encode(String contents, BarcodeFormat format, int width, int height) throws WriterException {
        return encode(contents, format, width, height, null);
    }

    @Override // com.google.zxing.Writer
    public BitMatrix encode(String contents, BarcodeFormat format, int width, int height, Map<EncodeHintType, ?> hints) throws WriterException {
        if (contents.isEmpty()) {
            throw new IllegalArgumentException("Found empty contents");
        }
        if (format != BarcodeFormat.QR_CODE) {
            throw new IllegalArgumentException("Can only encode QR_CODE, but got ".concat(String.valueOf(format)));
        }
        if (width < 0 || height < 0) {
            throw new IllegalArgumentException("Requested dimensions are too small: " + width + 'x' + height);
        }
        ErrorCorrectionLevel errorCorrectionLevel = ErrorCorrectionLevel.L;
        int quietZone = 4;
        if (hints != null) {
            if (hints.containsKey(EncodeHintType.ERROR_CORRECTION)) {
                errorCorrectionLevel = ErrorCorrectionLevel.valueOf(hints.get(EncodeHintType.ERROR_CORRECTION).toString());
            }
            if (hints.containsKey(EncodeHintType.MARGIN)) {
                quietZone = Integer.parseInt(hints.get(EncodeHintType.MARGIN).toString());
            }
        }
        return renderResult(Encoder.encode(contents, errorCorrectionLevel, hints), width, height, quietZone);
    }

    private static BitMatrix renderResult(QRCode code, int width, int height, int quietZone) {
        ByteMatrix matrix = code.getMatrix();
        ByteMatrix input = matrix;
        if (matrix == null) {
            throw new IllegalStateException();
        }
        int inputWidth = input.getWidth();
        int inputHeight = input.getHeight();
        int qrWidth = (quietZone << 1) + inputWidth;
        int qrHeight = (quietZone << 1) + inputHeight;
        int outputWidth = Math.max(width, qrWidth);
        int outputHeight = Math.max(height, qrHeight);
        int multiple = Math.min(outputWidth / qrWidth, outputHeight / qrHeight);
        int leftPadding = (outputWidth - (inputWidth * multiple)) / 2;
        int topPadding = (outputHeight - (inputHeight * multiple)) / 2;
        BitMatrix output = new BitMatrix(outputWidth, outputHeight);
        int inputY = 0;
        int outputY = topPadding;
        while (inputY < inputHeight) {
            int inputX = 0;
            int inputHeight2 = inputHeight;
            int inputHeight3 = leftPadding;
            while (inputX < inputWidth) {
                int inputWidth2 = inputWidth;
                ByteMatrix input2 = input;
                if (input.get(inputX, inputY) == 1) {
                    output.setRegion(inputHeight3, outputY, multiple, multiple);
                }
                inputX++;
                inputHeight3 += multiple;
                inputWidth = inputWidth2;
                input = input2;
            }
            inputY++;
            outputY += multiple;
            inputHeight = inputHeight2;
        }
        return output;
    }
}
