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
final class UPCEANExtension2Support {
    private final int[] decodeMiddleCounters = new int[4];
    private final StringBuilder decodeRowStringBuffer = new StringBuilder();

    UPCEANExtension2Support() {
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
        int checkParity = 0;
        for (int x = 0; x < 2 && rowOffset < end; x++) {
            int bestMatch = UPCEANReader.decodeDigit(row, counters, rowOffset, UPCEANReader.L_AND_G_PATTERNS);
            resultString.append((char) ((bestMatch % 10) + 48));
            for (int counter : counters) {
                rowOffset += counter;
            }
            if (bestMatch >= 10) {
                checkParity |= 1 << (1 - x);
            }
            if (x != 1) {
                rowOffset = row.getNextUnset(row.getNextSet(rowOffset));
            }
        }
        if (resultString.length() != 2) {
            throw NotFoundException.getNotFoundInstance();
        }
        if (Integer.parseInt(resultString.toString()) % 4 != checkParity) {
            throw NotFoundException.getNotFoundInstance();
        }
        return rowOffset;
    }

    private static Map<ResultMetadataType, Object> parseExtensionString(String raw) {
        if (raw.length() != 2) {
            return null;
        }
        Map<ResultMetadataType, Object> result = new EnumMap<>(ResultMetadataType.class);
        result.put(ResultMetadataType.ISSUE_NUMBER, Integer.valueOf(raw));
        return result;
    }
}
