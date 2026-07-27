package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.S4;
import com.android.tools.r8.internal.AbstractC1308bC;
import com.android.tools.r8.internal.AbstractC3644zm0;
import com.android.tools.r8.internal.C0874Ot;
import com.android.tools.r8.internal.C1631ec;
import com.android.tools.r8.internal.C2052iw;
import com.android.tools.r8.internal.C2867rd;
import com.android.tools.r8.internal.H5;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public interface Y {
    A5 a(A5 a5, AbstractC1308bC abstractC1308bC);

    C2867rd a(AbstractC1308bC abstractC1308bC, C2867rd c2867rd);

    S a(C0874Ot c0874Ot, AbstractC1308bC abstractC1308bC, S4.c cVar, A5 a5, A5 a52, C1631ec c1631ec, C2052iw c2052iw, AbstractC3644zm0 abstractC3644zm0);

    void a(C0874Ot c0874Ot);

    boolean a();

    /* JADX WARN: Removed duplicated region for block: B:10:0x0048  */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    default boolean a(com.android.tools.r8.graph.C0421y r3, com.android.tools.r8.ir.optimize.P r4, com.android.tools.r8.internal.AbstractC1308bC r5, com.android.tools.r8.graph.A5 r6, com.android.tools.r8.graph.A5 r7) {
        /*
            r2 = this;
            boolean r0 = r5.X1()
            if (r0 == 0) goto L48
            com.android.tools.r8.internal.cC r0 = r5.d0()
            com.android.tools.r8.internal.ul0 r0 = r0.V2()
            com.android.tools.r8.internal.vj0 r1 = r0.t()
            boolean r1 = r1.w()
            if (r1 != 0) goto L1d
            com.android.tools.r8.graph.D2 r5 = r6.a()
            goto L49
        L1d:
            com.android.tools.r8.internal.vj0 r0 = r0.t()
            com.android.tools.r8.internal.rd r0 = r0.b()
            com.android.tools.r8.internal.rd r5 = r2.a(r5, r0)
            com.android.tools.r8.graph.I2 r0 = r6.s()
            com.android.tools.r8.graph.y r1 = r2.b()
            com.android.tools.r8.internal.vj0 r0 = r0.b(r1)
            com.android.tools.r8.internal.rd r0 = r0.b()
            com.android.tools.r8.graph.y r1 = r2.b()
            boolean r5 = r5.b(r0, r1)
            if (r5 != 0) goto L48
            com.android.tools.r8.graph.D2 r5 = r6.a()
            goto L49
        L48:
            r5 = 0
        L49:
            if (r5 == 0) goto L5f
            com.android.tools.r8.graph.h r6 = r3.g()
            com.android.tools.r8.graph.j r6 = (com.android.tools.r8.graph.C0285j) r6
            com.android.tools.r8.internal.pV r3 = com.android.tools.r8.graph.AbstractC0232e.a(r5, r7, r3, r6)
            boolean r3 = r3.b()
            if (r3 == 0) goto L5d
            r3 = 0
            return r3
        L5d:
            r4.a = r5
        L5f:
            r3 = 1
            return r3
        */
        throw new UnsupportedOperationException("Method not decompiled: com.android.tools.r8.ir.optimize.Y.a(com.android.tools.r8.graph.y, com.android.tools.r8.ir.optimize.P, com.android.tools.r8.internal.bC, com.android.tools.r8.graph.A5, com.android.tools.r8.graph.A5):boolean");
    }

    boolean a(C0874Ot c0874Ot, S4.c cVar, A5 a5, AbstractC3644zm0 abstractC3644zm0);

    boolean a(Q q, C0874Ot c0874Ot, C0874Ot c0874Ot2, AbstractC1308bC abstractC1308bC, H5 h5, AbstractC3644zm0 abstractC3644zm0);

    boolean a(Q q, AbstractC3644zm0 abstractC3644zm0);

    C0421y b();
}
