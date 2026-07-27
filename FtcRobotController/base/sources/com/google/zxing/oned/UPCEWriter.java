package com.google.zxing.oned;

import com.google.zxing.BarcodeFormat;
import com.google.zxing.FormatException;
import java.util.Collection;
import java.util.Collections;

/* JADX INFO: loaded from: classes8.dex */
public final class UPCEWriter extends UPCEANWriter {
    private static final int CODE_WIDTH = 51;

    @Override // com.google.zxing.oned.OneDimensionalCodeWriter
    protected Collection<BarcodeFormat> getSupportedWriteFormats() {
        return Collections.singleton(BarcodeFormat.UPC_E);
    }

    @Override // com.google.zxing.oned.OneDimensionalCodeWriter
    public boolean[] encode(String str) {
        int length = str.length();
        switch (length) {
            case 7:
                try {
                    str = str + UPCEANReader.getStandardUPCEANChecksum(UPCEReader.convertUPCEtoUPCA(str));
                } catch (FormatException e) {
                    throw new IllegalArgumentException(e);
                }
                break;
            case 8:
                try {
                    if (!UPCEANReader.checkStandardUPCEANChecksum(UPCEReader.convertUPCEtoUPCA(str))) {
                        throw new IllegalArgumentException("Contents do not pass checksum");
                    }
                } catch (FormatException e2) {
                    throw new IllegalArgumentException("Illegal contents");
                }
                break;
            default:
                throw new IllegalArgumentException("Requested contents should be 7 or 8 digits long, but got ".concat(String.valueOf(length)));
        }
        checkNumeric(str);
        int iDigit = Character.digit(str.charAt(0), 10);
        if (iDigit != 0 && iDigit != 1) {
            throw new IllegalArgumentException("Number system must be 0 or 1");
        }
        int i = UPCEReader.NUMSYS_AND_CHECK_DIGIT_PATTERNS[iDigit][Character.digit(str.charAt(7), 10)];
        boolean[] zArr = new boolean[51];
        int iAppendPattern = appendPattern(zArr, 0, UPCEANReader.START_END_PATTERN, true);
        for (int i2 = 1; i2 <= 6; i2++) {
            int iDigit2 = Character.digit(str.charAt(i2), 10);
            if (((i >> (6 - i2)) & 1) == 1) {
                iDigit2 += 10;
            }
            iAppendPattern += appendPattern(zArr, iAppendPattern, UPCEANReader.L_AND_G_PATTERNS[iDigit2], false);
        }
        appendPattern(zArr, iAppendPattern, UPCEANReader.END_PATTERN, false);
        return zArr;
    }
}
