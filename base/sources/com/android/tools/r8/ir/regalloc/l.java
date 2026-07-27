package com.android.tools.r8.ir.regalloc;

import java.util.BitSet;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class l extends j {
    public static final /* synthetic */ boolean c = true;
    public final j a;
    public final BitSet b;

    public l(k kVar) {
        this.a = kVar;
        this.b = new BitSet(kVar.a);
    }

    @Override // com.android.tools.r8.ir.regalloc.j
    public final boolean a(int i, i iVar) {
        if (c || !this.b.get(i)) {
            return this.a.a(i, iVar);
        }
        throw new AssertionError();
    }

    @Override // com.android.tools.r8.ir.regalloc.j
    public final boolean b(int i) {
        return this.a.b(i) || this.b.get(i);
    }

    public final void c(int i) {
        this.b.set(i);
    }

    @Override // com.android.tools.r8.ir.regalloc.j
    public final int a(int i) {
        if (!c && this.b.get(i)) {
            throw new AssertionError();
        }
        return this.a.a(i);
    }
}
