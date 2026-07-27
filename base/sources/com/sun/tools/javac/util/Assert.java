package com.sun.tools.javac.util;

/* JADX INFO: loaded from: classes.dex */
public class Assert {
    public static void check(boolean cond) {
        if (!cond) {
            error();
        }
    }

    public static void checkNull(Object o) {
        if (o != null) {
            error();
        }
    }

    public static <T> T checkNonNull(T t) {
        if (t == null) {
            error();
        }
        return t;
    }

    public static void check(boolean cond, int value) {
        if (!cond) {
            error(String.valueOf(value));
        }
    }

    public static void check(boolean cond, long value) {
        if (!cond) {
            error(String.valueOf(value));
        }
    }

    public static void check(boolean cond, Object value) {
        if (!cond) {
            error(String.valueOf(value));
        }
    }

    public static void check(boolean cond, String msg) {
        if (!cond) {
            error(msg);
        }
    }

    public static void checkNull(Object o, Object value) {
        if (o != null) {
            error(String.valueOf(value));
        }
    }

    public static void checkNull(Object o, String msg) {
        if (o != null) {
            error(msg);
        }
    }

    public static <T> T checkNonNull(T t, String msg) {
        if (t == null) {
            error(msg);
        }
        return t;
    }

    public static void error() {
        throw new AssertionError();
    }

    public static void error(String msg) {
        throw new AssertionError(msg);
    }

    private Assert() {
    }
}
