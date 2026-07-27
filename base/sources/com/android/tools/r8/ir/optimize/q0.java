package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.internal.AbstractC0610Gd;
import com.android.tools.r8.internal.C0874Ot;
import com.android.tools.r8.internal.InterfaceC0710Jd;
import com.android.tools.r8.internal.JO;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class q0 extends AbstractC0610Gd {
    public static final /* synthetic */ int e = 0;

    public q0(C0421y c0421y) {
        super(c0421y);
    }

    @Override // com.android.tools.r8.internal.AbstractC0610Gd
    public final boolean a(C0874Ot c0874Ot, JO jo) {
        return this.a.M().b0 && (c0874Ot.i.b(6) || c0874Ot.i.b() || c0874Ot.i.b(27));
    }

    @Override // com.android.tools.r8.internal.AbstractC0610Gd
    public final String b() {
        return "RedundantFieldLoadAndStoreElimination";
    }

    @Override // com.android.tools.r8.internal.AbstractC0610Gd
    public final InterfaceC0710Jd b(C0874Ot c0874Ot) {
        return new p0(this, c0874Ot).c();
    }
}
