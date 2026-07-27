package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.AbstractC1308bC;
import com.android.tools.r8.internal.C0874Ot;
import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.internal.H5;
import com.android.tools.r8.internal.K5;
import com.android.tools.r8.internal.L5;

/* JADX INFO: renamed from: com.android.tools.r8.ir.optimize.t, reason: case insensitive filesystem */
/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public abstract class AbstractC3699t {
    public static final /* synthetic */ boolean a = true;

    public static void a(C0421y c0421y, C0874Ot c0874Ot) {
        AbstractC1308bC abstractC1308bCC0;
        com.android.tools.r8.graph.H0 h0G;
        if (c0421y.o()) {
            C0421y<?> c0421yU = c0421y.U();
            L5 l5T = c0874Ot.t();
            while (l5T.c.hasNext()) {
                H5 h5 = (H5) l5T.c.next();
                l5T.d = h5;
                K5 k5A = h5.a(c0874Ot);
                while (k5A.hasNext()) {
                    AbstractC1076Vw next = k5A.next();
                    if (next.W1() && (h0G = (abstractC1308bCC0 = next.c0()).g(c0421yU, c0874Ot.i())) != null && h0G.D().y()) {
                        C3161ul0 c3161ul0S2 = abstractC1308bCC0.S2();
                        if (abstractC1308bCC0.f1()) {
                            abstractC1308bCC0.d().f(c3161ul0S2);
                        }
                        if (c3161ul0S2.t().N().d()) {
                            k5A.p();
                        } else {
                            k5A.a(c0421yU, c3161ul0S2);
                        }
                    }
                }
            }
            if (!a && !c0874Ot.b((C0421y<?>) c0421y)) {
                throw new AssertionError();
            }
        }
    }
}
