package com.google.zxing.client.result;

import com.google.zxing.Result;

/* JADX INFO: loaded from: classes8.dex */
public final class URLTOResultParser extends ResultParser {
    @Override // com.google.zxing.client.result.ResultParser
    public URIParsedResult parse(Result result) {
        int titleEnd;
        String rawText = getMassagedText(result);
        if ((!rawText.startsWith("urlto:") && !rawText.startsWith("URLTO:")) || (titleEnd = rawText.indexOf(58, 6)) < 0) {
            return null;
        }
        String title = titleEnd > 6 ? rawText.substring(6, titleEnd) : null;
        String uri = rawText.substring(titleEnd + 1);
        return new URIParsedResult(uri, title);
    }
}
