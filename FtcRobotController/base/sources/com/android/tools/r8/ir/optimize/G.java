package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.AbstractC0324n0;
import com.android.tools.r8.graph.B1;
import com.android.tools.r8.graph.C0409w2;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.E2;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class G extends AbstractC0324n0 {
    public G(C0421y c0421y, A5 a5) {
        super(c0421y, a5);
    }

    @Override // com.android.tools.r8.graph.AbstractC0324n0, com.android.tools.r8.graph.Y5
    public final void h(C0409w2 c0409w2) {
        B1 b1A = this.a.a();
        b1A.getClass();
        if (c0409w2.w0().I0()) {
            E2 e2C0 = c0409w2.C0();
            if (b1A.f0.b(c0409w2.x0()) && e2C0.q0().isEmpty() && b1A.a2.a(e2C0.r0())) {
                this.e = Boolean.TRUE;
                a();
            }
        }
    }
}
