package com.google.gson.internal;

import com.google.gson.stream.JsonReader;
import java.io.IOException;

/* JADX INFO: loaded from: classes8.dex */
public abstract class JsonReaderInternalAccess {
    public static JsonReaderInternalAccess INSTANCE;

    public abstract void promoteNameToValue(JsonReader jsonReader) throws IOException;
}
