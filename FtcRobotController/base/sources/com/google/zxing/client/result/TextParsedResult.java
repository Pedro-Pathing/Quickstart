package com.google.zxing.client.result;

/* JADX INFO: loaded from: classes8.dex */
public final class TextParsedResult extends ParsedResult {
    private final String language;
    private final String text;

    public TextParsedResult(String text, String language) {
        super(ParsedResultType.TEXT);
        this.text = text;
        this.language = language;
    }

    public String getText() {
        return this.text;
    }

    public String getLanguage() {
        return this.language;
    }

    @Override // com.google.zxing.client.result.ParsedResult
    public String getDisplayResult() {
        return this.text;
    }
}
