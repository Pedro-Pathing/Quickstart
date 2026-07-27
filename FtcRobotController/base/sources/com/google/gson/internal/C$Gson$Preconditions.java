package com.google.gson.internal;

/* JADX INFO: renamed from: com.google.gson.internal.$Gson$Preconditions, reason: invalid class name */
/* JADX INFO: loaded from: classes8.dex */
public final class C$Gson$Preconditions {
    private C$Gson$Preconditions() {
        throw new UnsupportedOperationException();
    }

    public static <T> T checkNotNull(T obj) {
        if (obj == null) {
            throw new NullPointerException();
        }
        return obj;
    }

    public static void checkArgument(boolean condition) {
        if (!condition) {
            throw new IllegalArgumentException();
        }
    }
}
