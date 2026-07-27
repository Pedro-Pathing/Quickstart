package com.google.zxing.client.result;

import com.google.zxing.Result;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/* JADX INFO: loaded from: classes8.dex */
public final class URIResultParser extends ResultParser {
    private static final Pattern ALLOWED_URI_CHARS_PATTERN = Pattern.compile("[-._~:/?#\\[\\]@!$&'()*+,;=%A-Za-z0-9]+");
    private static final Pattern USER_IN_HOST = Pattern.compile(":/*([^/@]+)@[^/]+");
    private static final Pattern URL_WITH_PROTOCOL_PATTERN = Pattern.compile("[a-zA-Z][a-zA-Z0-9+-.]+:");
    private static final Pattern URL_WITHOUT_PROTOCOL_PATTERN = Pattern.compile("([a-zA-Z0-9\\-]+\\.){1,6}[a-zA-Z]{2,}(:\\d{1,5})?(/|\\?|$)");

    @Override // com.google.zxing.client.result.ResultParser
    public URIParsedResult parse(Result result) {
        String rawText = getMassagedText(result);
        if (rawText.startsWith("URL:") || rawText.startsWith("URI:")) {
            return new URIParsedResult(rawText.substring(4).trim(), null);
        }
        String rawText2 = rawText.trim();
        if (!isBasicallyValidURI(rawText2) || isPossiblyMaliciousURI(rawText2)) {
            return null;
        }
        return new URIParsedResult(rawText2, null);
    }

    static boolean isPossiblyMaliciousURI(String uri) {
        return !ALLOWED_URI_CHARS_PATTERN.matcher(uri).matches() || USER_IN_HOST.matcher(uri).find();
    }

    static boolean isBasicallyValidURI(String uri) {
        if (uri.contains(" ")) {
            return false;
        }
        Matcher m = URL_WITH_PROTOCOL_PATTERN.matcher(uri);
        if (m.find() && m.start() == 0) {
            return true;
        }
        Matcher m2 = URL_WITHOUT_PROTOCOL_PATTERN.matcher(uri);
        return m2.find() && m2.start() == 0;
    }
}
