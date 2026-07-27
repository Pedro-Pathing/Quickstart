package com.android.tools.r8.ir.regalloc;

import com.android.tools.r8.graph.C0286j0;
import com.android.tools.r8.internal.C3161ul0;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class a implements Comparable {
    public static final /* synthetic */ boolean g = true;
    public final C3161ul0 b;
    public final C0286j0 c;
    public final int d;
    public final int e;
    public final int f;

    public a(C3161ul0 c3161ul0, int i, int i2, int i3) {
        if (!g && !c3161ul0.y()) {
            throw new AssertionError();
        }
        this.b = c3161ul0;
        this.c = c3161ul0.r();
        this.d = i;
        this.e = i2;
        this.f = i3;
    }

    @Override // java.lang.Comparable
    /* JADX INFO: renamed from: a, reason: merged with bridge method [inline-methods] */
    public final int compareTo(a aVar) {
        int i = this.e;
        int i2 = aVar.e;
        return i != i2 ? Integer.compare(i, i2) : Integer.compare(this.f, aVar.f);
    }

    public final String toString() {
        return this.c + " @ r" + this.d + ": " + new e(this.e, this.f);
    }
}
