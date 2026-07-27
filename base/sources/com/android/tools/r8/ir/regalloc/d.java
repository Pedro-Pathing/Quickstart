package com.android.tools.r8.ir.regalloc;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public class d implements Comparable<d> {
    public final int b;
    public final int c;

    public d(int i, int i2) {
        this.b = i;
        this.c = i2;
    }

    @Override // java.lang.Comparable
    public final int compareTo(d dVar) {
        d dVar2 = dVar;
        int i = dVar2.b;
        int i2 = this.b;
        return i != i2 ? i2 - i : this.c - dVar2.c;
    }

    public final boolean equals(Object obj) {
        if (!(obj instanceof d)) {
            return false;
        }
        d dVar = (d) obj;
        return dVar.b == this.b && dVar.c == this.c;
    }

    public final int hashCode() {
        return (this.c * 7) + this.b;
    }
}
