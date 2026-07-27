package com.sun.tools.javac.util;

/* JADX INFO: loaded from: classes.dex */
public class PropagatedException extends RuntimeException {
    static final long serialVersionUID = -6065309339888775367L;

    public PropagatedException(RuntimeException cause) {
        super(cause);
    }

    @Override // java.lang.Throwable
    public RuntimeException getCause() {
        return (RuntimeException) super.getCause();
    }
}
