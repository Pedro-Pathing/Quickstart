package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.internal.AbstractC0695Iu;
import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.AbstractC2125ji0;
import com.android.tools.r8.internal.C0874Ot;
import com.android.tools.r8.internal.C1411cA;
import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.internal.S40;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public abstract class E0 {
    public static final C0 a;
    public static final /* synthetic */ boolean b = true;

    static {
        int i = AbstractC0695Iu.c;
        S40 s40 = S40.e;
        a = new C0(3, s40, s40);
    }

    public static C0 a(C0874Ot c0874Ot, y0 y0Var) {
        D0 d0 = new D0();
        AbstractC2125ji0 abstractC2125ji0A = new x0(new C1411cA(), y0Var, d0).a(c0874Ot.j());
        if (abstractC2125ji0A.c()) {
            d0.c = 3;
        } else {
            B0 b0 = (B0) abstractC2125ji0A.b().e();
            d0.c = b0.a;
            d0.b = b0.b;
        }
        return A0.a(d0.c) ? a : new C0(d0.c, d0.a, d0.b);
    }

    public static C0 a(final C0421y c0421y, C0874Ot c0874Ot) {
        if (!b && !c0874Ot.i().e().C1()) {
            throw new AssertionError();
        }
        final C3161ul0 c3161ul0O = c0874Ot.o();
        if (!c3161ul0O.O()) {
            return a;
        }
        final A5 a5I = c0874Ot.i();
        return a(c0874Ot, new y0() { // from class: com.android.tools.r8.ir.optimize.E0$$ExternalSyntheticLambda0
            @Override // com.android.tools.r8.ir.optimize.y0
            public final int a(AbstractC1076Vw abstractC1076Vw) {
                return E0.a(c3161ul0O, c0421y, a5I, abstractC1076Vw);
            }
        });
    }

    public static int a(C3161ul0 c3161ul0, C0421y c0421y, A5 a5, AbstractC1076Vw abstractC1076Vw) {
        return ((abstractC1076Vw.X1() && abstractC1076Vw.d0().V2() == c3161ul0) || (abstractC1076Vw.O1() && abstractC1076Vw.V().h() == c3161ul0) || (abstractC1076Vw.h2() && ((C3161ul0) abstractC1076Vw.m0().c.get(0)) == c3161ul0)) ? !abstractC1076Vw.b().x() ? 2 : 3 : abstractC1076Vw.b(c0421y, a5) ? 3 : 1;
    }

    public static C0 a(final C0421y c0421y, final A5 a5, final C0874Ot c0874Ot) {
        if (b || c0874Ot.i().e().z0()) {
            return a(c0874Ot, new y0() { // from class: com.android.tools.r8.ir.optimize.E0$$ExternalSyntheticLambda1
                @Override // com.android.tools.r8.ir.optimize.y0
                public final int a(AbstractC1076Vw abstractC1076Vw) {
                    return E0.a(c0874Ot, a5, c0421y, abstractC1076Vw);
                }
            });
        }
        throw new AssertionError();
    }

    public static int a(C0874Ot c0874Ot, A5 a5, C0421y c0421y, AbstractC1076Vw abstractC1076Vw) {
        return abstractC1076Vw.a(c0874Ot.i().s(), a5, c0421y, 1, 1) ? !abstractC1076Vw.b().x() ? 2 : 3 : (abstractC1076Vw.X1() || abstractC1076Vw.b(c0421y, a5)) ? 3 : 1;
    }
}
