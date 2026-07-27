package com.google.zxing.datamatrix;

import com.google.zxing.BarcodeFormat;
import com.google.zxing.Dimension;
import com.google.zxing.EncodeHintType;
import com.google.zxing.Writer;
import com.google.zxing.common.BitMatrix;
import com.google.zxing.datamatrix.encoder.DefaultPlacement;
import com.google.zxing.datamatrix.encoder.ErrorCorrection;
import com.google.zxing.datamatrix.encoder.HighLevelEncoder;
import com.google.zxing.datamatrix.encoder.SymbolInfo;
import com.google.zxing.datamatrix.encoder.SymbolShapeHint;
import com.google.zxing.qrcode.encoder.ByteMatrix;
import java.util.Map;

/* JADX INFO: loaded from: classes8.dex */
public final class DataMatrixWriter implements Writer {
    @Override // com.google.zxing.Writer
    public BitMatrix encode(String contents, BarcodeFormat format, int width, int height) {
        return encode(contents, format, width, height, null);
    }

    @Override // com.google.zxing.Writer
    public BitMatrix encode(String str, BarcodeFormat barcodeFormat, int i, int i2, Map<EncodeHintType, ?> map) {
        Dimension dimension;
        if (str.isEmpty()) {
            throw new IllegalArgumentException("Found empty contents");
        }
        if (barcodeFormat != BarcodeFormat.DATA_MATRIX) {
            throw new IllegalArgumentException("Can only encode DATA_MATRIX, but got ".concat(String.valueOf(barcodeFormat)));
        }
        if (i < 0 || i2 < 0) {
            throw new IllegalArgumentException("Requested dimensions can't be negative: " + i + 'x' + i2);
        }
        SymbolShapeHint symbolShapeHint = SymbolShapeHint.FORCE_NONE;
        Dimension dimension2 = null;
        if (map == null) {
            dimension = null;
        } else {
            SymbolShapeHint symbolShapeHint2 = (SymbolShapeHint) map.get(EncodeHintType.DATA_MATRIX_SHAPE);
            if (symbolShapeHint2 != null) {
                symbolShapeHint = symbolShapeHint2;
            }
            Dimension dimension3 = (Dimension) map.get(EncodeHintType.MIN_SIZE);
            if (dimension3 == null) {
                dimension3 = null;
            }
            dimension = (Dimension) map.get(EncodeHintType.MAX_SIZE);
            if (dimension == null) {
                dimension = null;
                dimension2 = dimension3;
            } else {
                dimension2 = dimension3;
            }
        }
        String strEncodeHighLevel = HighLevelEncoder.encodeHighLevel(str, symbolShapeHint, dimension2, dimension);
        SymbolInfo symbolInfoLookup = SymbolInfo.lookup(strEncodeHighLevel.length(), symbolShapeHint, dimension2, dimension, true);
        DefaultPlacement defaultPlacement = new DefaultPlacement(ErrorCorrection.encodeECC200(strEncodeHighLevel, symbolInfoLookup), symbolInfoLookup.getSymbolDataWidth(), symbolInfoLookup.getSymbolDataHeight());
        defaultPlacement.place();
        return encodeLowLevel(defaultPlacement, symbolInfoLookup, i, i2);
    }

    private static BitMatrix encodeLowLevel(DefaultPlacement placement, SymbolInfo symbolInfo, int width, int height) {
        int symbolWidth = symbolInfo.getSymbolDataWidth();
        int symbolHeight = symbolInfo.getSymbolDataHeight();
        ByteMatrix matrix = new ByteMatrix(symbolInfo.getSymbolWidth(), symbolInfo.getSymbolHeight());
        int matrixY = 0;
        for (int y = 0; y < symbolHeight; y++) {
            if (y % symbolInfo.matrixHeight == 0) {
                int matrixX = 0;
                for (int x = 0; x < symbolInfo.getSymbolWidth(); x++) {
                    matrix.set(matrixX, matrixY, x % 2 == 0);
                    matrixX++;
                }
                matrixY++;
            }
            int matrixX2 = 0;
            for (int x2 = 0; x2 < symbolWidth; x2++) {
                if (x2 % symbolInfo.matrixWidth == 0) {
                    matrix.set(matrixX2, matrixY, true);
                    matrixX2++;
                }
                matrix.set(matrixX2, matrixY, placement.getBit(x2, y));
                matrixX2++;
                if (x2 % symbolInfo.matrixWidth == symbolInfo.matrixWidth - 1) {
                    matrix.set(matrixX2, matrixY, y % 2 == 0);
                    matrixX2++;
                }
            }
            matrixY++;
            if (y % symbolInfo.matrixHeight == symbolInfo.matrixHeight - 1) {
                int matrixX3 = 0;
                for (int x3 = 0; x3 < symbolInfo.getSymbolWidth(); x3++) {
                    matrix.set(matrixX3, matrixY, true);
                    matrixX3++;
                }
                matrixY++;
            }
        }
        return convertByteMatrixToBitMatrix(matrix, width, height);
    }

    private static BitMatrix convertByteMatrixToBitMatrix(ByteMatrix matrix, int reqWidth, int reqHeight) {
        BitMatrix output;
        int matrixWidth = matrix.getWidth();
        int matrixHeight = matrix.getHeight();
        int outputWidth = Math.max(reqWidth, matrixWidth);
        int outputHeight = Math.max(reqHeight, matrixHeight);
        int multiple = Math.min(outputWidth / matrixWidth, outputHeight / matrixHeight);
        int leftPadding = (outputWidth - (matrixWidth * multiple)) / 2;
        int topPadding = (outputHeight - (matrixHeight * multiple)) / 2;
        if (reqHeight >= matrixHeight && reqWidth >= matrixWidth) {
            output = new BitMatrix(reqWidth, reqHeight);
        } else {
            leftPadding = 0;
            topPadding = 0;
            output = new BitMatrix(matrixWidth, matrixHeight);
        }
        output.clear();
        int inputY = 0;
        int outputY = topPadding;
        while (inputY < matrixHeight) {
            int inputX = 0;
            int outputX = leftPadding;
            while (inputX < matrixWidth) {
                if (matrix.get(inputX, inputY) == 1) {
                    output.setRegion(outputX, outputY, multiple, multiple);
                }
                inputX++;
                outputX += multiple;
            }
            inputY++;
            outputY += multiple;
        }
        return output;
    }
}
