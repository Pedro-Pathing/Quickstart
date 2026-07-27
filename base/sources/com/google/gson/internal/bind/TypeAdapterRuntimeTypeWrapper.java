package com.google.gson.internal.bind;

import com.google.gson.Gson;
import com.google.gson.TypeAdapter;
import com.google.gson.internal.bind.ReflectiveTypeAdapterFactory;
import com.google.gson.reflect.TypeToken;
import com.google.gson.stream.JsonReader;
import com.google.gson.stream.JsonWriter;
import java.io.IOException;
import java.lang.reflect.Type;
import java.lang.reflect.TypeVariable;

/* JADX INFO: loaded from: classes8.dex */
final class TypeAdapterRuntimeTypeWrapper<T> extends TypeAdapter<T> {
    private final Gson context;
    private final TypeAdapter<T> delegate;
    private final Type type;

    TypeAdapterRuntimeTypeWrapper(Gson context, TypeAdapter<T> delegate, Type type) {
        this.context = context;
        this.delegate = delegate;
        this.type = type;
    }

    @Override // com.google.gson.TypeAdapter
    /* JADX INFO: renamed from: read */
    public T read2(JsonReader in) throws IOException {
        return this.delegate.read2(in);
    }

    @Override // com.google.gson.TypeAdapter
    public void write(JsonWriter out, T value) throws IOException {
        TypeAdapter<T> typeAdapter = this.delegate;
        Type runtimeType = getRuntimeTypeIfMoreSpecific(this.type, value);
        if (runtimeType != this.type) {
            TypeAdapter<T> adapter = this.context.getAdapter(TypeToken.get(runtimeType));
            if ((adapter instanceof ReflectiveTypeAdapterFactory.Adapter) && !(this.delegate instanceof ReflectiveTypeAdapterFactory.Adapter)) {
                typeAdapter = this.delegate;
            } else {
                typeAdapter = adapter;
            }
        }
        typeAdapter.write(out, value);
    }

    private Type getRuntimeTypeIfMoreSpecific(Type type, Object value) {
        if (value == null) {
            return type;
        }
        if (type == Object.class || (type instanceof TypeVariable) || (type instanceof Class)) {
            return value.getClass();
        }
        return type;
    }
}
