package com.google.zxing;

/* JADX INFO: loaded from: classes8.dex */
public abstract class ReaderException extends Exception {
    protected static final StackTraceElement[] NO_TRACE;
    protected static final boolean isStackTrace;

    static {
        isStackTrace = System.getProperty("surefire.test.class.path") != null;
        NO_TRACE = new StackTraceElement[0];
    }

    ReaderException() {
    }

    ReaderException(Throwable cause) {
        super(cause);
    }

    @Override // java.lang.Throwable
    public final synchronized Throwable fillInStackTrace() {
        return null;
    }
}
