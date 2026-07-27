package com.google.zxing.oned;

import com.google.zxing.BarcodeFormat;
import java.util.Collection;
import java.util.Collections;

/* JADX INFO: loaded from: classes8.dex */
public final class ITFWriter extends OneDimensionalCodeWriter {
    private static final int N = 1;
    private static final int W = 3;
    private static final int[] START_PATTERN = {1, 1, 1, 1};
    private static final int[] END_PATTERN = {3, 1, 1};
    private static final int[][] PATTERNS = {new int[]{1, 1, 3, 3, 1}, new int[]{3, 1, 1, 1, 3}, new int[]{1, 3, 1, 1, 3}, new int[]{3, 3, 1, 1, 1}, new int[]{1, 1, 3, 1, 3}, new int[]{3, 1, 3, 1, 1}, new int[]{1, 3, 3, 1, 1}, new int[]{1, 1, 1, 3, 3}, new int[]{3, 1, 1, 3, 1}, new int[]{1, 3, 1, 3, 1}};

    @Override // com.google.zxing.oned.OneDimensionalCodeWriter
    protected Collection<BarcodeFormat> getSupportedWriteFormats() {
        return Collections.singleton(BarcodeFormat.ITF);
    }

    @Override // com.google.zxing.oned.OneDimensionalCodeWriter
    public boolean[] encode(String contents) {
        int length = contents.length();
        if (length % 2 == 0) {
            if (length > 80) {
                throw new IllegalArgumentException("Requested contents should be less than 80 digits long, but got ".concat(String.valueOf(length)));
            }
            checkNumeric(contents);
            boolean[] result = new boolean[(length * 9) + 9];
            int pos = appendPattern(result, 0, START_PATTERN, true);
            for (int i = 0; i < length; i += 2) {
                int one = Character.digit(contents.charAt(i), 10);
                int two = Character.digit(contents.charAt(i + 1), 10);
                int[] encoding = new int[10];
                for (int j = 0; j < 5; j++) {
                    encoding[j * 2] = PATTERNS[one][j];
                    encoding[(j * 2) + 1] = PATTERNS[two][j];
                }
                int j2 = appendPattern(result, pos, encoding, true);
                pos += j2;
            }
            appendPattern(result, pos, END_PATTERN, true);
            return result;
        }
        throw new IllegalArgumentException("The length of the input should be even");
    }
}
