package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.AbstractC3250vj0;
import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.internal.K5;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class n0 implements m0 {
    public final C3161ul0 a;
    public final /* synthetic */ p0 b;

    public n0(p0 p0Var, C3161ul0 c3161ul0) {
        this.b = p0Var;
        this.a = c3161ul0;
    }

    @Override // com.android.tools.r8.ir.optimize.m0
    public final void a(K5 k5, AbstractC1076Vw abstractC1076Vw) {
        abstractC1076Vw.d().a(this.a, this.b.d);
        k5.p();
        this.b.h = true;
    }

    public final String toString() {
        return "ExistingValue(v" + this.a.s() + ")";
    }

    @Override // com.android.tools.r8.ir.optimize.m0
    public final AbstractC3250vj0 a(C0421y c0421y, AbstractC3250vj0 abstractC3250vj0) {
        return this.a.t();
    }
}
