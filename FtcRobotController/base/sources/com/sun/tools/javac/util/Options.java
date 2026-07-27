package com.sun.tools.javac.util;

import com.sun.tools.javac.main.Option;
import com.sun.tools.javac.util.Context;
import java.util.LinkedHashMap;
import java.util.Set;

/* JADX INFO: loaded from: classes.dex */
public class Options {
    public static final Context.Key<Options> optionsKey = new Context.Key<>();
    private static final long serialVersionUID = 0;
    private List<Runnable> listeners = List.nil();
    private LinkedHashMap<String, String> values = new LinkedHashMap<>();

    public static Options instance(Context context) {
        Options instance = (Options) context.get(optionsKey);
        if (instance == null) {
            return new Options(context);
        }
        return instance;
    }

    protected Options(Context context) {
        context.put(optionsKey, this);
    }

    public String get(String name) {
        return this.values.get(name);
    }

    public String get(Option option) {
        return this.values.get(option.text);
    }

    public boolean getBoolean(String name) {
        return getBoolean(name, false);
    }

    public boolean getBoolean(String name, boolean defaultValue) {
        String value = get(name);
        return value == null ? defaultValue : Boolean.parseBoolean(value);
    }

    public boolean isSet(String name) {
        return this.values.get(name) != null;
    }

    public boolean isSet(Option option) {
        return this.values.get(option.text) != null;
    }

    public boolean isSet(Option option, String value) {
        return this.values.get(new StringBuilder().append(option.text).append(value).toString()) != null;
    }

    public boolean isUnset(String name) {
        return this.values.get(name) == null;
    }

    public boolean isUnset(Option option) {
        return this.values.get(option.text) == null;
    }

    public boolean isUnset(Option option, String value) {
        return this.values.get(new StringBuilder().append(option.text).append(value).toString()) == null;
    }

    public void put(String name, String value) {
        this.values.put(name, value);
    }

    public void put(Option option, String value) {
        this.values.put(option.text, value);
    }

    public void putAll(Options options) {
        this.values.putAll(options.values);
    }

    public void remove(String name) {
        this.values.remove(name);
    }

    public Set<String> keySet() {
        return this.values.keySet();
    }

    public int size() {
        return this.values.size();
    }

    public void addListener(Runnable listener) {
        this.listeners = this.listeners.prepend(listener);
    }

    public void notifyListeners() {
        for (Runnable r : this.listeners) {
            r.run();
        }
    }

    public boolean lint(String s) {
        return isSet(Option.XLINT_CUSTOM, s) || ((isSet(Option.XLINT) || isSet(Option.XLINT_CUSTOM, "all")) && isUnset(Option.XLINT_CUSTOM, new StringBuilder().append("-").append(s).toString()));
    }
}
