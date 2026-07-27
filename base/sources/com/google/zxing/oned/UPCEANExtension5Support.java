package com.google.zxing.oned;

import com.google.zxing.BarcodeFormat;
import com.google.zxing.NotFoundException;
import com.google.zxing.Result;
import com.google.zxing.ResultMetadataType;
import com.google.zxing.ResultPoint;
import com.google.zxing.common.BitArray;
import java.util.EnumMap;
import java.util.Map;

/* JADX INFO: loaded from: classes8.dex */
final class UPCEANExtension5Support {
    private static final int[] CHECK_DIGIT_ENCODINGS = {24, 20, 18, 17, 12, 6, 3, 10, 9, 5};
    private final int[] decodeMiddleCounters = new int[4];
    private final StringBuilder decodeRowStringBuffer = new StringBuilder();

    UPCEANExtension5Support() {
    }

    Result decodeRow(int rowNumber, BitArray row, int[] extensionStartRange) throws NotFoundException {
        StringBuilder result = this.decodeRowStringBuffer;
        result.setLength(0);
        int end = decodeMiddle(row, extensionStartRange, result);
        String resultString = result.toString();
        Map<ResultMetadataType, Object> extensionData = parseExtensionString(resultString);
        Result extensionResult = new Result(resultString, null, new ResultPoint[]{new ResultPoint((extensionStartRange[0] + extensionStartRange[1]) / 2.0f, rowNumber), new ResultPoint(end, rowNumber)}, BarcodeFormat.UPC_EAN_EXTENSION);
        if (extensionData != null) {
            extensionResult.putAllMetadata(extensionData);
        }
        return extensionResult;
    }

    private int decodeMiddle(BitArray row, int[] startRange, StringBuilder resultString) throws NotFoundException {
        int[] counters = this.decodeMiddleCounters;
        counters[0] = 0;
        counters[1] = 0;
        counters[2] = 0;
        counters[3] = 0;
        int end = row.getSize();
        int rowOffset = startRange[1];
        int lgPatternFound = 0;
        for (int x = 0; x < 5 && rowOffset < end; x++) {
            int bestMatch = UPCEANReader.decodeDigit(row, counters, rowOffset, UPCEANReader.L_AND_G_PATTERNS);
            resultString.append((char) ((bestMatch % 10) + 48));
            for (int counter : counters) {
                rowOffset += counter;
            }
            if (bestMatch >= 10) {
                lgPatternFound |= 1 << (4 - x);
            }
            if (x != 4) {
                rowOffset = row.getNextUnset(row.getNextSet(rowOffset));
            }
        }
        if (resultString.length() != 5) {
            throw NotFoundException.getNotFoundInstance();
        }
        int checkDigit = determineCheckDigit(lgPatternFound);
        if (extensionChecksum(resultString.toString()) != checkDigit) {
            throw NotFoundException.getNotFoundInstance();
        }
        return rowOffset;
    }

    private static int extensionChecksum(CharSequence s) {
        int length = s.length();
        int sum = 0;
        for (int i = length - 2; i >= 0; i -= 2) {
            sum += s.charAt(i) - '0';
        }
        int sum2 = sum * 3;
        for (int i2 = length - 1; i2 >= 0; i2 -= 2) {
            sum2 += s.charAt(i2) - '0';
        }
        int i3 = sum2 * 3;
        return i3 % 10;
    }

    private static int determineCheckDigit(int lgPatternFound) throws NotFoundException {
        for (int d = 0; d < 10; d++) {
            if (lgPatternFound == CHECK_DIGIT_ENCODINGS[d]) {
                return d;
            }
        }
        throw NotFoundException.getNotFoundInstance();
    }

    private static Map<ResultMetadataType, Object> parseExtensionString(String raw) {
        Object extension5String;
        if (raw.length() != 5 || (extension5String = parseExtension5String(raw)) == null) {
            return null;
        }
        Map<ResultMetadataType, Object> result = new EnumMap<>(ResultMetadataType.class);
        result.put(ResultMetadataType.SUGGESTED_PRICE, extension5String);
        return result;
    }

    /* JADX WARN: Can't fix incorrect switch cases order, some code will duplicate */
    /* JADX WARN: Removed duplicated region for block: B:17:0x0031  */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    private static java.lang.String parseExtension5String(java.lang.String r4) {
        /*
            r0 = 0
            char r1 = r4.charAt(r0)
            r2 = 1
            java.lang.String r3 = ""
            switch(r1) {
                case 48: goto L43;
                case 53: goto L3f;
                case 57: goto Lc;
                default: goto Lb;
            }
        Lb:
            goto L46
        Lc:
            int r1 = r4.hashCode()
            switch(r1) {
                case 54118329: goto L28;
                case 54395376: goto L1e;
                case 54395377: goto L14;
                default: goto L13;
            }
        L13:
            goto L31
        L14:
            java.lang.String r0 = "99991"
            boolean r0 = r4.equals(r0)
            if (r0 == 0) goto L13
            r0 = r2
            goto L32
        L1e:
            java.lang.String r0 = "99990"
            boolean r0 = r4.equals(r0)
            if (r0 == 0) goto L13
            r0 = 2
            goto L32
        L28:
            java.lang.String r1 = "90000"
            boolean r1 = r4.equals(r1)
            if (r1 == 0) goto L13
            goto L32
        L31:
            r0 = -1
        L32:
            switch(r0) {
                case 0: goto L3d;
                case 1: goto L3a;
                case 2: goto L37;
                default: goto L36;
            }
        L36:
            goto L46
        L37:
            java.lang.String r4 = "Used"
            return r4
        L3a:
            java.lang.String r4 = "0.00"
            return r4
        L3d:
            r4 = 0
            return r4
        L3f:
            java.lang.String r3 = "$"
            goto L46
        L43:
            java.lang.String r3 = "£"
        L46:
            java.lang.String r4 = r4.substring(r2)
            int r4 = java.lang.Integer.parseInt(r4)
            int r0 = r4 / 100
            java.lang.String r0 = java.lang.String.valueOf(r0)
            int r4 = r4 % 100
            r1 = 10
            if (r4 >= r1) goto L65
            java.lang.String r1 = "0"
            java.lang.String r4 = java.lang.String.valueOf(r4)
            java.lang.String r4 = r1.concat(r4)
            goto L69
        L65:
            java.lang.String r4 = java.lang.String.valueOf(r4)
        L69:
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r1.<init>()
            java.lang.StringBuilder r1 = r1.append(r3)
            java.lang.StringBuilder r0 = r1.append(r0)
            r1 = 46
            java.lang.StringBuilder r0 = r0.append(r1)
            java.lang.StringBuilder r4 = r0.append(r4)
            java.lang.String r4 = r4.toString()
            return r4
        */
        throw new UnsupportedOperationException("Method not decompiled: com.google.zxing.oned.UPCEANExtension5Support.parseExtension5String(java.lang.String):java.lang.String");
    }
}
