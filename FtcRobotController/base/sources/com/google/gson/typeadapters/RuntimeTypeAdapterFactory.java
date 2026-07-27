package com.google.gson.typeadapters;

import com.google.gson.Gson;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParseException;
import com.google.gson.JsonPrimitive;
import com.google.gson.TypeAdapter;
import com.google.gson.TypeAdapterFactory;
import com.google.gson.internal.Streams;
import com.google.gson.reflect.TypeToken;
import com.google.gson.stream.JsonReader;
import com.google.gson.stream.JsonWriter;
import com.qualcomm.ftccommon.configuration.RobotConfigResFilter;
import java.io.IOException;
import java.util.LinkedHashMap;
import java.util.Map;

/* JADX INFO: loaded from: classes8.dex */
public final class RuntimeTypeAdapterFactory<T> implements TypeAdapterFactory {
    private final Class<?> baseType;
    private final Map<String, Class<?>> labelToSubtype = new LinkedHashMap();
    private final Map<Class<?>, String> subtypeToLabel = new LinkedHashMap();
    private final String typeFieldName;

    private RuntimeTypeAdapterFactory(Class<?> baseType, String typeFieldName) {
        if (typeFieldName == null || baseType == null) {
            throw new NullPointerException();
        }
        this.baseType = baseType;
        this.typeFieldName = typeFieldName;
    }

    public static <T> RuntimeTypeAdapterFactory<T> of(Class<T> baseType, String typeFieldName) {
        return new RuntimeTypeAdapterFactory<>(baseType, typeFieldName);
    }

    public static <T> RuntimeTypeAdapterFactory<T> of(Class<T> baseType) {
        return new RuntimeTypeAdapterFactory<>(baseType, RobotConfigResFilter.robotConfigRootTypeAttribute);
    }

    public RuntimeTypeAdapterFactory<T> registerSubtype(Class<? extends T> type, String label) {
        if (type == null || label == null) {
            throw new NullPointerException();
        }
        if (this.subtypeToLabel.containsKey(type) || this.labelToSubtype.containsKey(label)) {
            throw new IllegalArgumentException("types and labels must be unique");
        }
        this.labelToSubtype.put(label, type);
        this.subtypeToLabel.put(type, label);
        return this;
    }

    public RuntimeTypeAdapterFactory<T> registerSubtype(Class<? extends T> type) {
        return registerSubtype(type, type.getSimpleName());
    }

    @Override // com.google.gson.TypeAdapterFactory
    public <R> TypeAdapter<R> create(Gson gson, TypeToken<R> type) {
        if (type.getRawType() != this.baseType) {
            return null;
        }
        final Map<String, TypeAdapter<?>> labelToDelegate = new LinkedHashMap<>();
        final Map<Class<?>, TypeAdapter<?>> subtypeToDelegate = new LinkedHashMap<>();
        for (Map.Entry<String, Class<?>> entry : this.labelToSubtype.entrySet()) {
            TypeAdapter<T> delegateAdapter = gson.getDelegateAdapter(this, TypeToken.get((Class) entry.getValue()));
            labelToDelegate.put(entry.getKey(), delegateAdapter);
            subtypeToDelegate.put(entry.getValue(), delegateAdapter);
        }
        return new TypeAdapter<R>() { // from class: com.google.gson.typeadapters.RuntimeTypeAdapterFactory.1
            @Override // com.google.gson.TypeAdapter
            /* JADX INFO: renamed from: read */
            public R read2(JsonReader in) throws IOException {
                JsonElement jsonElement = Streams.parse(in);
                JsonElement labelJsonElement = jsonElement.getAsJsonObject().remove(RuntimeTypeAdapterFactory.this.typeFieldName);
                if (labelJsonElement == null) {
                    throw new JsonParseException("cannot deserialize " + RuntimeTypeAdapterFactory.this.baseType + " because it does not define a field named " + RuntimeTypeAdapterFactory.this.typeFieldName);
                }
                String label = labelJsonElement.getAsString();
                TypeAdapter<R> delegate = (TypeAdapter) labelToDelegate.get(label);
                if (delegate == null) {
                    throw new JsonParseException("cannot deserialize " + RuntimeTypeAdapterFactory.this.baseType + " subtype named " + label + "; did you forget to register a subtype?");
                }
                return delegate.fromJsonTree(jsonElement);
            }

            @Override // com.google.gson.TypeAdapter
            public void write(JsonWriter out, R value) throws IOException {
                Class<?> srcType = value.getClass();
                String label = (String) RuntimeTypeAdapterFactory.this.subtypeToLabel.get(srcType);
                TypeAdapter<R> delegate = (TypeAdapter) subtypeToDelegate.get(srcType);
                if (delegate == null) {
                    throw new JsonParseException("cannot serialize " + srcType.getName() + "; did you forget to register a subtype?");
                }
                JsonObject jsonObject = delegate.toJsonTree(value).getAsJsonObject();
                if (jsonObject.has(RuntimeTypeAdapterFactory.this.typeFieldName)) {
                    throw new JsonParseException("cannot serialize " + srcType.getName() + " because it already defines a field named " + RuntimeTypeAdapterFactory.this.typeFieldName);
                }
                JsonObject clone = new JsonObject();
                clone.add(RuntimeTypeAdapterFactory.this.typeFieldName, new JsonPrimitive(label));
                for (Map.Entry<String, JsonElement> e : jsonObject.entrySet()) {
                    clone.add(e.getKey(), e.getValue());
                }
                Streams.write(clone, out);
            }
        }.nullSafe();
    }
}
