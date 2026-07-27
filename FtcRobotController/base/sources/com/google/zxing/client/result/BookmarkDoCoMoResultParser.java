package com.google.zxing.client.result;

import com.google.zxing.Result;

/* JADX INFO: loaded from: classes8.dex */
public final class BookmarkDoCoMoResultParser extends AbstractDoCoMoResultParser {
    @Override // com.google.zxing.client.result.ResultParser
    public URIParsedResult parse(Result result) {
        String rawText = result.getText();
        if (!rawText.startsWith("MEBKM:")) {
            return null;
        }
        String title = matchSingleDoCoMoPrefixedField("TITLE:", rawText, true);
        String[] rawUri = matchDoCoMoPrefixedField("URL:", rawText);
        if (rawUri == null) {
            return null;
        }
        String uri = rawUri[0];
        if (URIResultParser.isBasicallyValidURI(uri)) {
            return new URIParsedResult(uri, title);
        }
        return null;
    }
}
