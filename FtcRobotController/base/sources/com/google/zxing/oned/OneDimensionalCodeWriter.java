package com.google.zxing.oned;

import com.google.zxing.BarcodeFormat;
import com.google.zxing.EncodeHintType;
import com.google.zxing.Writer;
import com.google.zxing.common.BitMatrix;
import java.util.Collection;
import java.util.Map;
import java.util.regex.Pattern;

/* JADX INFO: loaded from: classes8.dex */
public abstract class OneDimensionalCodeWriter implements Writer {
    private static final Pattern NUMERIC = Pattern.compile("[0-9]+");

    public abstract boolean[] encode(String str);

    @Override // com.google.zxing.Writer
    public final BitMatrix encode(String contents, BarcodeFormat format, int width, int height) {
        return encode(contents, format, width, height, null);
    }

    @Override // com.google.zxing.Writer
    public BitMatrix encode(String contents, BarcodeFormat format, int width, int height, Map<EncodeHintType, ?> hints) {
        if (contents.isEmpty()) {
            throw new IllegalArgumentException("Found empty contents");
        }
        if (width < 0 || height < 0) {
            throw new IllegalArgumentException("Negative size is not allowed. Input: " + width + 'x' + height);
        }
        Collection<BarcodeFormat> supportedFormats = getSupportedWriteFormats();
        if (supportedFormats != null && !supportedFormats.contains(format)) {
            throw new IllegalArgumentException("Can only encode " + supportedFormats + ", but got " + format);
        }
        int sidesMargin = getDefaultMargin();
        if (hints != null && hints.containsKey(EncodeHintType.MARGIN)) {
            sidesMargin = Integer.parseInt(hints.get(EncodeHintType.MARGIN).toString());
        }
        return renderResult(encode(contents), width, height, sidesMargin);
    }

    protected Collection<BarcodeFormat> getSupportedWriteFormats() {
        return null;
    }

    private static BitMatrix renderResult(boolean[] code, int width, int height, int sidesMargin) {
        int inputWidth = code.length;
        int fullWidth = inputWidth + sidesMargin;
        int outputWidth = Math.max(width, fullWidth);
        int outputHeight = Math.max(1, height);
        int multiple = outputWidth / fullWidth;
        int leftPadding = (outputWidth - (inputWidth * multiple)) / 2;
        BitMatrix output = new BitMatrix(outputWidth, outputHeight);
        int inputX = 0;
        int outputX = leftPadding;
        while (inputX < inputWidth) {
            if (code[inputX]) {
                output.setRegion(outputX, 0, multiple, outputHeight);
            }
            inputX++;
            outputX += multiple;
        }
        return output;
    }

    protected static void checkNumeric(String contents) {
        if (!NUMERIC.matcher(contents).matches()) {
            throw new IllegalArgumentException("Input should only contain digits 0-9");
        }
    }

    protected static int appendPattern(boolean[] target, int pos, int[] pattern, boolean startColor) {
        boolean color = startColor;
        int numAdded = 0;
        for (int len : pattern) {
            int j = 0;
            while (j < len) {
                target[pos] = color;
                j++;
                pos++;
            }
            numAdded += len;
            color = !color;
        }
        return numAdded;
    }

    public int getDefaultMargin() {
        return 10;
    }
}
