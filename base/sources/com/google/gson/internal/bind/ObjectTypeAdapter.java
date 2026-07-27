package com.google.gson.internal.bind;

import com.google.gson.Gson;
import com.google.gson.TypeAdapter;
import com.google.gson.TypeAdapterFactory;
import com.google.gson.internal.LinkedTreeMap;
import com.google.gson.reflect.TypeToken;
import com.google.gson.stream.JsonReader;
import com.google.gson.stream.JsonToken;
import com.google.gson.stream.JsonWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/* JADX INFO: loaded from: classes8.dex */
public final class ObjectTypeAdapter extends TypeAdapter<Object> {
    public static final TypeAdapterFactory FACTORY = new TypeAdapterFactory() { // from class: com.google.gson.internal.bind.ObjectTypeAdapter.1
        @Override // com.google.gson.TypeAdapterFactory
        public <T> TypeAdapter<T> create(Gson gson, TypeToken<T> type) {
            if (type.getRawType() == Object.class) {
                return new ObjectTypeAdapter(gson);
            }
            return null;
        }
    };
    private final Gson gson;

    ObjectTypeAdapter(Gson gson) {
        this.gson = gson;
    }

    @Override // com.google.gson.TypeAdapter
    /* JADX INFO: renamed from: read */
    public Object read2(JsonReader in) throws IOException {
        JsonToken token = in.peek();
        switch (token) {
            case BEGIN_ARRAY:
                List<Object> list = new ArrayList<>();
                in.beginArray();
                while (in.hasNext()) {
                    list.add(read2(in));
                }
                in.endArray();
                return list;
            case BEGIN_OBJECT:
                Map<String, Object> map = new LinkedTreeMap<>();
                in.beginObject();
                while (in.hasNext()) {
                    map.put(in.nextName(), read2(in));
                }
                in.endObject();
                return map;
            case STRING:
                return in.nextString();
            case NUMBER:
                return Double.valueOf(in.nextDouble());
            case BOOLEAN:
                return Boolean.valueOf(in.nextBoolean());
            case NULL:
                in.nextNull();
                return null;
            default:
                throw new IllegalStateException();
        }
    }

    @Override // com.google.gson.TypeAdapter
    public void write(JsonWriter out, Object value) throws IOException {
        if (value == null) {
            out.nullValue();
            return;
        }
        TypeAdapter<Object> typeAdapter = this.gson.getAdapter(value.getClass());
        if (typeAdapter instanceof ObjectTypeAdapter) {
            out.beginObject();
            out.endObject();
        } else {
            typeAdapter.write(out, value);
        }
    }
}
