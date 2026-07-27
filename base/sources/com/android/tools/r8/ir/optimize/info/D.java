package com.android.tools.r8.ir.optimize.info;

import com.android.tools.r8.K;
import com.android.tools.r8.graph.C0257g1;
import com.android.tools.r8.graph.C0285j;
import com.android.tools.r8.graph.C0291j1;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.D2;
import com.android.tools.r8.internal.AbstractC0564Em;
import com.android.tools.r8.internal.AbstractC2670pV;
import com.android.tools.r8.internal.C1092Wm;
import com.android.tools.r8.internal.C1395c2;
import com.android.tools.r8.internal.C2700pk0;
import com.android.tools.r8.internal.C2789qk0;
import com.android.tools.r8.internal.Dk0;
import com.android.tools.r8.internal.FQ;
import java.util.Iterator;
import java.util.concurrent.ExecutorService;
import java.util.function.Consumer;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public abstract class D {
    public static void a(C0421y c0421y, ExecutorService executorService) {
        K.a(c0421y, ((C0285j) c0421y.g()).d(), new Consumer() { // from class: com.android.tools.r8.ir.optimize.info.D$$ExternalSyntheticLambda0
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                D.a((D2) obj);
            }
        }, executorService);
    }

    public static void a(D2 d2) {
        Iterator<C0257g1> it = d2.M0().iterator();
        while (it.hasNext()) {
            v vVarC = it.next().l.c();
            if (vVarC != null) {
                vVarC.a = Dk0.a;
                vVarC.d = AbstractC0564Em.m();
            }
        }
        for (C0291j1 c0291j1 : d2.C1()) {
            c0291j1.P0();
            w wVarA = c0291j1.m.a();
            if (wVarA != null) {
                wVarA.f = Dk0.a;
                E e = E.a;
                wVarA.b = e;
                wVarA.a(AbstractC0564Em.m());
                wVarA.c = C3682d.c;
                wVarA.m = C1092Wm.a;
                FQ fq = FQ.b;
                wVarA.p = fq;
                wVarA.q = fq;
                if (wVarA.b == e && wVarA.d == -1 && wVarA.e == C2700pk0.a && wVarA.f == C3682d.d && wVarA.g == C1395c2.a && !wVarA.h && wVarA.i == C2789qk0.a && wVarA.j == AbstractC0564Em.m() && wVarA.v == 3 && wVarA.k == AbstractC2670pV.c && wVarA.l == null && wVarA.m.c() && wVarA.n == null && wVarA.o == null && wVarA.p == fq && wVarA.q == fq && wVarA.r == 0 && wVarA.s == null && wVarA.t == null && wVarA.u == w.w) {
                    c0291j1.P0();
                    c0291j1.m = C3682d.b;
                }
            }
        }
    }
}
