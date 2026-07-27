package com.google.gson.graph;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.InstanceCreator;
import com.google.gson.JsonElement;
import com.google.gson.TypeAdapter;
import com.google.gson.TypeAdapterFactory;
import com.google.gson.internal.ConstructorConstructor;
import com.google.gson.internal.ObjectConstructor;
import com.google.gson.reflect.TypeToken;
import com.google.gson.stream.JsonReader;
import com.google.gson.stream.JsonToken;
import com.google.gson.stream.JsonWriter;
import java.io.IOException;
import java.lang.reflect.Type;
import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.Queue;

/* JADX INFO: loaded from: classes8.dex */
public final class GraphAdapterBuilder {
    private final Map<Type, InstanceCreator<?>> instanceCreators = new HashMap();
    private final ConstructorConstructor constructorConstructor = new ConstructorConstructor(this.instanceCreators);

    public GraphAdapterBuilder addType(Type type) {
        final ObjectConstructor<?> objectConstructor = this.constructorConstructor.get(TypeToken.get(type));
        return addType(type, new InstanceCreator<Object>() { // from class: com.google.gson.graph.GraphAdapterBuilder.1
            @Override // com.google.gson.InstanceCreator
            public Object createInstance(Type type2) {
                return objectConstructor.construct();
            }
        });
    }

    public GraphAdapterBuilder addType(Type type, InstanceCreator<?> instanceCreator) {
        if (type == null || instanceCreator == null) {
            throw new NullPointerException();
        }
        this.instanceCreators.put(type, instanceCreator);
        return this;
    }

    public void registerOn(GsonBuilder gsonBuilder) {
        Factory factory = new Factory(this.instanceCreators);
        gsonBuilder.registerTypeAdapterFactory(factory);
        for (Map.Entry<Type, InstanceCreator<?>> entry : this.instanceCreators.entrySet()) {
            gsonBuilder.registerTypeAdapter(entry.getKey(), factory);
        }
    }

    static class Factory implements TypeAdapterFactory, InstanceCreator {
        private final ThreadLocal<Graph> graphThreadLocal = new ThreadLocal<>();
        private final Map<Type, InstanceCreator<?>> instanceCreators;

        Factory(Map<Type, InstanceCreator<?>> instanceCreators) {
            this.instanceCreators = instanceCreators;
        }

        @Override // com.google.gson.TypeAdapterFactory
        public <T> TypeAdapter<T> create(Gson gson, TypeToken<T> type) {
            if (!this.instanceCreators.containsKey(type.getType())) {
                return null;
            }
            final TypeAdapter<T> typeAdapter = gson.getDelegateAdapter(this, type);
            final TypeAdapter<T> adapter = gson.getAdapter(JsonElement.class);
            return new TypeAdapter<T>() { // from class: com.google.gson.graph.GraphAdapterBuilder.Factory.1
                @Override // com.google.gson.TypeAdapter
                public void write(JsonWriter out, T value) throws IOException {
                    if (value != null) {
                        Graph graph = (Graph) Factory.this.graphThreadLocal.get();
                        boolean writeEntireGraph = false;
                        if (graph == null) {
                            writeEntireGraph = true;
                            graph = new Graph(new IdentityHashMap());
                        }
                        Element<T> element = (Element) graph.map.get(value);
                        if (element == null) {
                            element = new Element<>(value, graph.nextName(), typeAdapter, null);
                            graph.map.put(value, element);
                            graph.queue.add(element);
                        }
                        if (writeEntireGraph) {
                            Factory.this.graphThreadLocal.set(graph);
                            try {
                                out.beginObject();
                                while (true) {
                                    Element<?> current = (Element) graph.queue.poll();
                                    if (current == null) {
                                        out.endObject();
                                        return;
                                    } else {
                                        out.name(((Element) current).id);
                                        current.write(out);
                                    }
                                }
                            } finally {
                                Factory.this.graphThreadLocal.remove();
                            }
                        } else {
                            out.value(((Element) element).id);
                        }
                    } else {
                        out.nullValue();
                    }
                }

                @Override // com.google.gson.TypeAdapter
                /* JADX INFO: renamed from: read */
                public T read2(JsonReader jsonReader) throws IOException {
                    if (jsonReader.peek() == JsonToken.NULL) {
                        jsonReader.nextNull();
                        return null;
                    }
                    String strNextString = null;
                    Graph graph = (Graph) Factory.this.graphThreadLocal.get();
                    boolean z = false;
                    if (graph == null) {
                        graph = new Graph(new HashMap());
                        z = true;
                        jsonReader.beginObject();
                        while (jsonReader.hasNext()) {
                            String strNextName = jsonReader.nextName();
                            if (strNextString == null) {
                                strNextString = strNextName;
                            }
                            graph.map.put(strNextName, new Element(null, strNextName, typeAdapter, (JsonElement) adapter.read2(jsonReader)));
                        }
                        jsonReader.endObject();
                    } else {
                        strNextString = jsonReader.nextString();
                    }
                    if (z) {
                        Factory.this.graphThreadLocal.set(graph);
                    }
                    try {
                        Element element = (Element) graph.map.get(strNextString);
                        if (element.value == null) {
                            element.typeAdapter = typeAdapter;
                            element.read(graph);
                        }
                        return (T) element.value;
                    } finally {
                        if (z) {
                            Factory.this.graphThreadLocal.remove();
                        }
                    }
                }
            };
        }

        @Override // com.google.gson.InstanceCreator
        public Object createInstance(Type type) {
            Graph graph = this.graphThreadLocal.get();
            if (graph == null || graph.nextCreate == null) {
                throw new IllegalStateException("Unexpected call to createInstance() for " + type);
            }
            InstanceCreator<?> creator = this.instanceCreators.get(type);
            Object result = creator.createInstance(type);
            graph.nextCreate.value = result;
            graph.nextCreate = null;
            return result;
        }
    }

    static class Graph {
        private final Map<Object, Element<?>> map;
        private Element nextCreate;
        private final Queue<Element> queue;

        private Graph(Map<Object, Element<?>> map) {
            this.queue = new LinkedList();
            this.map = map;
        }

        public String nextName() {
            return "0x" + Integer.toHexString(this.map.size() + 1);
        }
    }

    static class Element<T> {
        private final JsonElement element;
        private final String id;
        private TypeAdapter<T> typeAdapter;
        private T value;

        Element(T value, String id, TypeAdapter<T> typeAdapter, JsonElement element) {
            this.value = value;
            this.id = id;
            this.typeAdapter = typeAdapter;
            this.element = element;
        }

        void write(JsonWriter out) throws IOException {
            this.typeAdapter.write(out, this.value);
        }

        void read(Graph graph) throws IOException {
            if (graph.nextCreate == null) {
                graph.nextCreate = this;
                this.value = this.typeAdapter.fromJsonTree(this.element);
                if (this.value == null) {
                    throw new IllegalStateException("non-null value deserialized to null: " + this.element);
                }
                return;
            }
            throw new IllegalStateException("Unexpected recursive call to read() for " + this.id);
        }
    }
}
