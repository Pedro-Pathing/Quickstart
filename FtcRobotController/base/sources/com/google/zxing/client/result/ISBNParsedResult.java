package com.google.zxing.client.result;

/* JADX INFO: loaded from: classes8.dex */
public final class ISBNParsedResult extends ParsedResult {
    private final String isbn;

    ISBNParsedResult(String isbn) {
        super(ParsedResultType.ISBN);
        this.isbn = isbn;
    }

    public String getISBN() {
        return this.isbn;
    }

    @Override // com.google.zxing.client.result.ParsedResult
    public String getDisplayResult() {
        return this.isbn;
    }
}
