package com.android.tools.r8.ir.optimize;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public enum N {
    c("NEVER"),
    d("SAMECLASS"),
    e("SAMENEST"),
    f("PACKAGE"),
    g("SUBCLASS"),
    h("ALWAYS");

    public final int b;

    N(String str) {
        this.b = i;
    }

    public final boolean b(int i2) {
        return (i2 & this.b) != 0;
    }
}
