package com.google.zxing.client.result;

/* JADX INFO: loaded from: classes8.dex */
public final class URIParsedResult extends ParsedResult {
    private final String title;
    private final String uri;

    public URIParsedResult(String uri, String title) {
        super(ParsedResultType.URI);
        this.uri = massageURI(uri);
        this.title = title;
    }

    public String getURI() {
        return this.uri;
    }

    public String getTitle() {
        return this.title;
    }

    @Deprecated
    public boolean isPossiblyMaliciousURI() {
        return URIResultParser.isPossiblyMaliciousURI(this.uri);
    }

    @Override // com.google.zxing.client.result.ParsedResult
    public String getDisplayResult() {
        StringBuilder result = new StringBuilder(30);
        maybeAppend(this.title, result);
        maybeAppend(this.uri, result);
        return result.toString();
    }

    private static String massageURI(String uri) {
        String uri2 = uri.trim();
        int protocolEnd = uri2.indexOf(58);
        if (protocolEnd >= 0 && !isColonFollowedByPortNumber(uri2, protocolEnd)) {
            return uri2;
        }
        return "http://".concat(String.valueOf(uri2));
    }

    private static boolean isColonFollowedByPortNumber(String uri, int protocolEnd) {
        int start = protocolEnd + 1;
        int iIndexOf = uri.indexOf(47, start);
        int nextSlash = iIndexOf;
        if (iIndexOf < 0) {
            nextSlash = uri.length();
        }
        return ResultParser.isSubstringOfDigits(uri, start, nextSlash - start);
    }
}
