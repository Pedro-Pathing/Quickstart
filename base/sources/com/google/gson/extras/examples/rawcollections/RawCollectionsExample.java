package com.google.gson.extras.examples.rawcollections;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonParser;
import java.util.ArrayList;
import java.util.Collection;

/* JADX INFO: loaded from: classes8.dex */
public class RawCollectionsExample {

    static class Event {
        private String name;
        private String source;

        private Event(String name, String source) {
            this.name = name;
            this.source = source;
        }

        public String toString() {
            return String.format("(name=%s, source=%s)", this.name, this.source);
        }
    }

    public static void main(String[] args) {
        Gson gson = new Gson();
        Collection collection = new ArrayList();
        collection.add("hello");
        collection.add(5);
        collection.add(new Event("GREETINGS", "guest"));
        String json = gson.toJson(collection);
        System.out.println("Using Gson.toJson() on a raw collection: " + json);
        JsonParser parser = new JsonParser();
        JsonArray array = parser.parse(json).getAsJsonArray();
        String message = (String) gson.fromJson(array.get(0), String.class);
        int number = ((Integer) gson.fromJson(array.get(1), Integer.TYPE)).intValue();
        Event event = (Event) gson.fromJson(array.get(2), Event.class);
        System.out.printf("Using Gson.fromJson() to get: %s, %d, %s", message, Integer.valueOf(number), event);
    }
}
