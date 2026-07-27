package com.sun.tools.javac.util;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

/* JADX INFO: loaded from: classes.dex */
public class Context {
    private Map<Key<?>, Object> ht = new HashMap();
    private Map<Key<?>, Factory<?>> ft = new HashMap();
    private Map<Class<?>, Key<?>> kt = new HashMap();

    public interface Factory<T> {
        T make(Context context);
    }

    public static class Key<T> {
    }

    public <T> void put(Key<T> key, Factory<T> fac) {
        checkState(this.ht);
        Object old = this.ht.put(key, fac);
        if (old != null) {
            throw new AssertionError("duplicate context value");
        }
        checkState(this.ft);
        this.ft.put(key, fac);
    }

    public <T> void put(Key<T> key, T data) {
        if (data instanceof Factory) {
            throw new AssertionError("T extends Context.Factory");
        }
        checkState(this.ht);
        Object old = this.ht.put(key, data);
        if (old != null && !(old instanceof Factory) && old != data && data != null) {
            throw new AssertionError("duplicate context value");
        }
    }

    public <T> T get(Key<T> key) {
        checkState(this.ht);
        Object objMake = this.ht.get(key);
        if (objMake instanceof Factory) {
            objMake = ((Factory) objMake).make(this);
            if (objMake instanceof Factory) {
                throw new AssertionError("T extends Context.Factory");
            }
            Assert.check(this.ht.get(key) == objMake);
        }
        return (T) uncheckedCast(objMake);
    }

    public Context() {
    }

    public Context(Context prev) {
        this.kt.putAll(prev.kt);
        this.ft.putAll(prev.ft);
        this.ht.putAll(prev.ft);
    }

    private <T> Key<T> key(Class<T> clss) {
        checkState(this.kt);
        Key<T> k = (Key) uncheckedCast(this.kt.get(clss));
        if (k == null) {
            Key<T> k2 = new Key<>();
            this.kt.put(clss, k2);
            return k2;
        }
        return k;
    }

    public <T> T get(Class<T> cls) {
        return (T) get(key(cls));
    }

    public <T> void put(Class<T> clazz, T data) {
        put(key(clazz), data);
    }

    public <T> void put(Class<T> clazz, Factory<T> fac) {
        put((Key) key(clazz), (Factory) fac);
    }

    /* JADX WARN: Multi-variable type inference failed */
    private static <T> T uncheckedCast(Object obj) {
        return obj;
    }

    public void dump() {
        Iterator<Object> it = this.ht.values().iterator();
        while (it.hasNext()) {
            Object value = it.next();
            System.err.println(value == null ? null : value.getClass());
        }
    }

    public void clear() {
        this.ht = null;
        this.kt = null;
        this.ft = null;
    }

    private static void checkState(Map<?, ?> t) {
        if (t == null) {
            throw new IllegalStateException();
        }
    }
}
