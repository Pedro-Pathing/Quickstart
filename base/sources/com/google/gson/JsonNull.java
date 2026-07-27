package com.google.gson;

/* JADX INFO: loaded from: classes8.dex */
public final class JsonNull extends JsonElement {
    public static final JsonNull INSTANCE = new JsonNull();

    @Deprecated
    public JsonNull() {
    }

    /* JADX INFO: Access modifiers changed from: package-private */
    @Override // com.google.gson.JsonElement
    public JsonNull deepCopy() {
        return INSTANCE;
    }

    public int hashCode() {
        return JsonNull.class.hashCode();
    }

    public boolean equals(Object other) {
        return this == other || (other instanceof JsonNull);
    }
}
