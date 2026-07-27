package com.google.zxing.client.result;

/* JADX INFO: loaded from: classes8.dex */
public final class EmailAddressParsedResult extends ParsedResult {
    private final String[] bccs;
    private final String body;
    private final String[] ccs;
    private final String subject;
    private final String[] tos;

    EmailAddressParsedResult(String to) {
        this(new String[]{to}, null, null, null, null);
    }

    EmailAddressParsedResult(String[] tos, String[] ccs, String[] bccs, String subject, String body) {
        super(ParsedResultType.EMAIL_ADDRESS);
        this.tos = tos;
        this.ccs = ccs;
        this.bccs = bccs;
        this.subject = subject;
        this.body = body;
    }

    @Deprecated
    public String getEmailAddress() {
        if (this.tos == null || this.tos.length == 0) {
            return null;
        }
        return this.tos[0];
    }

    public String[] getTos() {
        return this.tos;
    }

    public String[] getCCs() {
        return this.ccs;
    }

    public String[] getBCCs() {
        return this.bccs;
    }

    public String getSubject() {
        return this.subject;
    }

    public String getBody() {
        return this.body;
    }

    @Deprecated
    public String getMailtoURI() {
        return "mailto:";
    }

    @Override // com.google.zxing.client.result.ParsedResult
    public String getDisplayResult() {
        StringBuilder result = new StringBuilder(30);
        maybeAppend(this.tos, result);
        maybeAppend(this.ccs, result);
        maybeAppend(this.bccs, result);
        maybeAppend(this.subject, result);
        maybeAppend(this.body, result);
        return result.toString();
    }
}
