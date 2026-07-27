package com.google.zxing.client.result;

/* JADX INFO: loaded from: classes8.dex */
public final class TelParsedResult extends ParsedResult {
    private final String number;
    private final String telURI;
    private final String title;

    public TelParsedResult(String number, String telURI, String title) {
        super(ParsedResultType.TEL);
        this.number = number;
        this.telURI = telURI;
        this.title = title;
    }

    public String getNumber() {
        return this.number;
    }

    public String getTelURI() {
        return this.telURI;
    }

    public String getTitle() {
        return this.title;
    }

    @Override // com.google.zxing.client.result.ParsedResult
    public String getDisplayResult() {
        StringBuilder result = new StringBuilder(20);
        maybeAppend(this.number, result);
        maybeAppend(this.title, result);
        return result.toString();
    }
}
