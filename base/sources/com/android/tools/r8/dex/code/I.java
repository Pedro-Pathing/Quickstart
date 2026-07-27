package com.android.tools.r8.dex.code;

import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.internal.AbstractC3250vj0;
import com.android.tools.r8.internal.AbstractC3650zs;
import com.android.tools.r8.internal.C0756Kt;
import com.android.tools.r8.internal.Jl0;
import com.android.tools.r8.internal.SJ;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes.dex */
public class I extends I0 {
    public I(int i, B1 b1) {
        super(i, b1);
    }

    @Override // com.android.tools.r8.dex.code.I0, com.android.tools.r8.dex.code.A1
    public final /* bridge */ /* synthetic */ void a(C0421y c0421y, AbstractC3650zs abstractC3650zs, com.android.tools.r8.dex.M m, A5 a5, SJ sj) {
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final boolean h() {
        return true;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String m() {
        return "ArrayLength";
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final int p() {
        return 33;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String u() {
        return "array-length";
    }

    public I(int i, int i2) {
        super(i, i2);
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final void a(C0756Kt c0756Kt) {
        byte b = this.f;
        c0756Kt.a(new com.android.tools.r8.internal.J3(c0756Kt.a(b, 2, AbstractC3250vj0.k()), c0756Kt.b(this.g, Jl0.b)));
    }
}
