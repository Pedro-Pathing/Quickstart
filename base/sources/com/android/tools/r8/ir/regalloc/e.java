package com.android.tools.r8.ir.regalloc;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public class e implements Comparable<e> {
    public static final e d = new e(0, Integer.MAX_VALUE);
    public final int b;
    public int c;

    public e(int i, int i2) {
        this.b = i;
        this.c = i2;
    }

    @Override // java.lang.Comparable
    public final int compareTo(e eVar) {
        e eVar2 = eVar;
        int i = this.b;
        int i2 = eVar2.b;
        return i != i2 ? i - i2 : this.c - eVar2.c;
    }

    public final String toString() {
        return "[" + this.b + ", " + this.c + "[";
    }
}
