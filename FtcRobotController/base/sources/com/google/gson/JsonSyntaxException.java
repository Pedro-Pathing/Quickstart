package com.google.gson;

/* JADX INFO: loaded from: classes8.dex */
public final class JsonSyntaxException extends JsonParseException {
    private static final long serialVersionUID = 1;

    public JsonSyntaxException(String msg) {
        super(msg);
    }

    public JsonSyntaxException(String msg, Throwable cause) {
        super(msg, cause);
    }

    public JsonSyntaxException(Throwable cause) {
        super(cause);
    }
}
