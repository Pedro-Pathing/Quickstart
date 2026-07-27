package com.google.zxing.client.result;

import com.google.zxing.BarcodeFormat;
import com.google.zxing.Result;

/* JADX INFO: loaded from: classes8.dex */
public final class ISBNResultParser extends ResultParser {
    @Override // com.google.zxing.client.result.ResultParser
    public ISBNParsedResult parse(Result result) {
        if (result.getBarcodeFormat() != BarcodeFormat.EAN_13) {
            return null;
        }
        String rawText = getMassagedText(result);
        if (rawText.length() != 13) {
            return null;
        }
        if (rawText.startsWith("978") || rawText.startsWith("979")) {
            return new ISBNParsedResult(rawText);
        }
        return null;
    }
}
