package com.android.tools.r8.dex.code;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes.dex */
public abstract class W {
    public static final /* synthetic */ boolean a = true;

    public static void a(A1 a1, A1 a12) {
        boolean z = a;
        if (!z && a1.getClass() != a12.getClass()) {
            throw new AssertionError();
        }
        if (!z && a1.k() != a12.k()) {
            throw new AssertionError();
        }
        if (!z && !a1.toString().equals(a12.toString())) {
            throw new AssertionError();
        }
    }
}
