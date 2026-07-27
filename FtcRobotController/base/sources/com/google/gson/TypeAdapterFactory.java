package com.google.gson;

import com.google.gson.reflect.TypeToken;

/* JADX INFO: loaded from: classes8.dex */
public interface TypeAdapterFactory {
    <T> TypeAdapter<T> create(Gson gson, TypeToken<T> typeToken);
}
