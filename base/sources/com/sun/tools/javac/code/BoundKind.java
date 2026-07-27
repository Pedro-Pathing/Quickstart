package com.sun.tools.javac.code;

/* JADX INFO: loaded from: classes.dex */
public enum BoundKind {
    EXTENDS("? extends "),
    SUPER("? super "),
    UNBOUND("?");

    private final String name;

    BoundKind(String name) {
        this.name = name;
    }

    @Override // java.lang.Enum
    public String toString() {
        return this.name;
    }
}
