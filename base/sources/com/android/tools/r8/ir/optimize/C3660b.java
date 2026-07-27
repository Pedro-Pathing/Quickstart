package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.AbstractC0275i0;
import com.android.tools.r8.graph.B1;
import com.android.tools.r8.graph.C0286j0;
import com.android.tools.r8.graph.C0409w2;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.E2;
import com.android.tools.r8.graph.E4;
import com.android.tools.r8.graph.I2;
import com.android.tools.r8.internal.AbstractC0564Em;
import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.AbstractC3006t5;
import com.android.tools.r8.internal.AbstractC3250vj0;
import com.android.tools.r8.internal.C0611Ge;
import com.android.tools.r8.internal.C0874Ot;
import com.android.tools.r8.internal.C2287lC;
import com.android.tools.r8.internal.C2384mC;
import com.android.tools.r8.internal.C2867rd;
import com.android.tools.r8.internal.C2881rj0;
import com.android.tools.r8.internal.C2947sS;
import com.android.tools.r8.internal.C2983so;
import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.internal.C3200vB;
import com.android.tools.r8.internal.CX;
import com.android.tools.r8.internal.EnumC3471y2;
import com.android.tools.r8.internal.JO;
import com.android.tools.r8.internal.K5;
import com.android.tools.r8.internal.L5;
import com.android.tools.r8.internal.LQ;
import com.android.tools.r8.internal.WB;
import com.android.tools.r8.synthesis.S;
import java.util.ArrayList;
import java.util.Set;
import java.util.function.Consumer;

/* JADX INFO: renamed from: com.android.tools.r8.ir.optimize.b, reason: case insensitive filesystem */
/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class C3660b {
    public static final /* synthetic */ boolean d = true;
    public final C0421y a;
    public final B1 b;
    public final C3200vB c;

    public C3660b(C0421y c0421y) {
        this.a = c0421y;
        this.c = c0421y.M();
        this.b = c0421y.a();
    }

    public final void a(final C0874Ot c0874Ot, JO jo, C0611Ge c0611Ge) {
        if (!d) {
            jo.getClass();
            if (jo instanceof CX) {
                throw new AssertionError();
            }
        }
        C3200vB c3200vB = this.c;
        c3200vB.getClass();
        if (c3200vB.b(EnumC3471y2.u)) {
            return;
        }
        C3658a c3658a = new C3658a();
        L5 l5T = c0874Ot.t();
        while (l5T.hasNext()) {
            K5 k5A = l5T.next().a(c0874Ot);
            ArrayList arrayList = new ArrayList();
            while (k5A.hasNext()) {
                WB wbA0 = k5A.next().a0();
                if (wbA0 != null && wbA0.U2().a(this.b.x4.a)) {
                    if (!d && wbA0.c.size() != 3) {
                        throw new AssertionError();
                    }
                    C3161ul0 c3161ul0V2 = wbA0.V2();
                    AbstractC1076Vw abstractC1076VwM = c3161ul0V2.m();
                    if (abstractC1076VwM.o2()) {
                        boolean z = C2384mC.m;
                        C2287lC c2287lC = new C2287lC();
                        c2287lC.d = a(jo, c0611Ge).getReference();
                        c2287lC.a = c0874Ot.a(this.b.n3.b(this.a), (C0286j0) null);
                        C2287lC c2287lC2 = (C2287lC) c2287lC.a();
                        c2287lC2.b = wbA0.getPosition();
                        C2384mC c2384mCC = ((C2287lC) c2287lC2.a(wbA0.c.subList(1, 3))).c();
                        k5A.a(c2384mCC, (Set<C3161ul0>) null);
                        c3161ul0V2.a(c2384mCC.d(), c3658a);
                        arrayList.add(abstractC1076VwM.u0());
                    }
                }
            }
            arrayList.forEach(new Consumer() { // from class: com.android.tools.r8.ir.optimize.b$$ExternalSyntheticLambda1
                @Override // java.util.function.Consumer
                public final void accept(Object obj) {
                    C3660b.a(c0874Ot, (LQ) obj);
                }
            });
        }
        C0421y c0421y = this.a;
        if (!c3658a.b.isEmpty()) {
            new C2881rj0(c0421y, c0874Ot, false).a(c3658a, 2);
        }
        if (!d && !c0874Ot.b(this.a)) {
            throw new AssertionError();
        }
    }

    public final A5 a(JO jo, C0611Ge c0611Ge) {
        final B1 b1A = this.a.a();
        final E2 e2A = b1A.a(b1A.n3, b1A.Y1, b1A.o3);
        A5 a5B = this.a.a.g().b(new com.android.tools.r8.synthesis.I() { // from class: com.android.tools.r8.ir.optimize.b$$ExternalSyntheticLambda2
            @Override // com.android.tools.r8.synthesis.I
            public final S.b a(com.android.tools.r8.synthesis.S s) {
                return s.y;
            }
        }, c0611Ge.a(), this.a, new Consumer() { // from class: com.android.tools.r8.ir.optimize.b$$ExternalSyntheticLambda3
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                this.f$0.a(e2A, b1A, (com.android.tools.r8.synthesis.N) obj);
            }
        });
        com.android.tools.r8.ir.optimize.info.C cA = com.android.tools.r8.ir.optimize.info.y.a();
        C0421y c0421y = this.a;
        I2 i2 = this.b.n3;
        C2947sS c2947sSB = C2947sS.b();
        i2.getClass();
        C2867rd c2867rdB = AbstractC3250vj0.a(i2, c2947sSB, (C0421y<?>) c0421y).b();
        boolean z = AbstractC0564Em.a;
        C2983so c2983so = new C2983so(c2867rdB);
        cA.getClass();
        cA.a(c0421y, a5B.e(), c2983so);
        jo.b(a5B);
        jo.d().f(a5B, c0611Ge.c);
        return a5B;
    }

    public final void a(E2 e2, final B1 b1, com.android.tools.r8.synthesis.N n) {
        n.m = this.a.U;
        n.e = e2;
        n.h = E4.b(4105, false);
        n.g = new com.android.tools.r8.synthesis.M() { // from class: com.android.tools.r8.ir.optimize.b$$ExternalSyntheticLambda0
            @Override // com.android.tools.r8.synthesis.M
            public final AbstractC0275i0 a(C0409w2 c0409w2) {
                return AbstractC3006t5.a(b1, c0409w2);
            }
        };
    }

    public static void a(C0874Ot c0874Ot, LQ lq) {
        lq.b().a(c0874Ot, lq).p();
    }
}
