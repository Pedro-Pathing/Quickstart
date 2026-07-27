package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.AbstractC2575oO;
import com.android.tools.r8.internal.AbstractC2614oo;
import com.android.tools.r8.internal.C0874Ot;
import com.android.tools.r8.internal.C3161ul0;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class Z extends AbstractC2614oo {
    public final com.android.tools.r8.ir.regalloc.f a;
    public final AbstractC2575oO.a b;

    public Z(C0874Ot c0874Ot, com.android.tools.r8.ir.regalloc.f fVar) {
        this.a = fVar;
        this.b = c0874Ot.b;
    }

    @Override // com.android.tools.r8.internal.AbstractC2614oo
    public final boolean a(Object obj, Object obj2) {
        AbstractC1076Vw abstractC1076Vw = (AbstractC1076Vw) obj;
        AbstractC1076Vw abstractC1076Vw2 = (AbstractC1076Vw) obj2;
        return abstractC1076Vw.a(abstractC1076Vw2, this.a, this.b) && abstractC1076Vw.b().i().equals(abstractC1076Vw2.b().i());
    }

    @Override // com.android.tools.r8.internal.AbstractC2614oo
    public final int a(Object obj) {
        int iA;
        AbstractC1076Vw abstractC1076Vw = (AbstractC1076Vw) obj;
        if (abstractC1076Vw.d() != null && abstractC1076Vw.d().T()) {
            iA = this.a.a(abstractC1076Vw.d(), abstractC1076Vw.e);
        } else {
            iA = 0;
        }
        for (C3161ul0 c3161ul0 : abstractC1076Vw.c) {
            iA <<= 4;
            if (c3161ul0.T()) {
                iA += this.a.a(c3161ul0, abstractC1076Vw.e);
            }
        }
        return abstractC1076Vw.b().i().hashCode() + (iA * 37);
    }
}
