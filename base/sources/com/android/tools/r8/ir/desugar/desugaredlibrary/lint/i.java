package com.android.tools.r8.ir.desugar.desugaredlibrary.lint;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public abstract class i {
    public final String a = System.lineSeparator();
    public final StringBuilder b = new StringBuilder();
    public String c = "";

    public final void a(String str) {
        this.b.append(str);
    }

    public final void b(String str) {
        this.b.append(this.c);
        this.b.append(str);
        this.b.append(this.a);
    }

    public final void c(String str) {
        this.b.append(str);
        this.b.append(this.a);
    }

    public final void d(String str) {
        this.b.append(this.c);
        this.b.append(str);
    }

    public final String toString() {
        return this.b.toString();
    }
}
