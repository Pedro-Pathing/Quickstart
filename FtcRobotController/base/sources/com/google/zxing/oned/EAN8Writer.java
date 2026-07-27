package com.google.zxing.oned;

import com.google.zxing.BarcodeFormat;
import com.google.zxing.FormatException;
import java.util.Collection;
import java.util.Collections;

/* JADX INFO: loaded from: classes8.dex */
public final class EAN8Writer extends UPCEANWriter {
    private static final int CODE_WIDTH = 67;

    @Override // com.google.zxing.oned.OneDimensionalCodeWriter
    protected Collection<BarcodeFormat> getSupportedWriteFormats() {
        return Collections.singleton(BarcodeFormat.EAN_8);
    }

    @Override // com.google.zxing.oned.OneDimensionalCodeWriter
    public boolean[] encode(String str) {
        int length = str.length();
        switch (length) {
            case 7:
                try {
                    str = str + UPCEANReader.getStandardUPCEANChecksum(str);
                } catch (FormatException e) {
                    throw new IllegalArgumentException(e);
                }
                break;
            case 8:
                try {
                    if (!UPCEANReader.checkStandardUPCEANChecksum(str)) {
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
        boolean[] zArr = new boolean[67];
        int iAppendPattern = appendPattern(zArr, 0, UPCEANReader.START_END_PATTERN, true) + 0;
        for (int i = 0; i <= 3; i++) {
            iAppendPattern += appendPattern(zArr, iAppendPattern, UPCEANReader.L_PATTERNS[Character.digit(str.charAt(i), 10)], false);
        }
        int iAppendPattern2 = iAppendPattern + appendPattern(zArr, iAppendPattern, UPCEANReader.MIDDLE_PATTERN, false);
        for (int i2 = 4; i2 <= 7; i2++) {
            iAppendPattern2 += appendPattern(zArr, iAppendPattern2, UPCEANReader.L_PATTERNS[Character.digit(str.charAt(i2), 10)], true);
        }
        appendPattern(zArr, iAppendPattern2, UPCEANReader.START_END_PATTERN, true);
        return zArr;
    }
}
