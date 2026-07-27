package com.android.tools.r8.ir.optimize.info;

import com.android.tools.r8.graph.B2;
import com.android.tools.r8.graph.C0285j;
import com.android.tools.r8.graph.C0291j1;
import com.android.tools.r8.graph.C0409w2;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.D2;
import com.android.tools.r8.graph.H0;
import com.android.tools.r8.graph.S4;
import com.android.tools.r8.graph.V3;
import com.android.tools.r8.internal.AbstractC3247vi;
import com.android.tools.r8.internal.C0918Ql;
import com.android.tools.r8.internal.C0945Rl;
import com.android.tools.r8.internal.HX;
import com.android.tools.r8.internal.JM;
import com.android.tools.r8.internal.KX;
import com.android.tools.r8.internal.PX;
import com.android.tools.r8.shaking.C3877i;
import java.util.HashMap;
import java.util.HashSet;
import java.util.IdentityHashMap;
import java.util.Iterator;
import java.util.function.Consumer;
import java.util.function.Predicate;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class n extends AbstractC3247vi {
    public static final /* synthetic */ boolean h = true;
    public final r f;
    public final IdentityHashMap g;

    public n(C0421y c0421y, r rVar, V3 v3) {
        super(c0421y, v3);
        this.g = new IdentityHashMap();
        this.f = rVar;
    }

    public final void a(m mVar, D2 d2) {
        mVar.a.a(((o) this.g.getOrDefault(d2, m.b)).a().a);
        for (C0291j1 c0291j1 : d2.G1()) {
            if (!c0291j1.p1()) {
                if (!c0291j1.l1()) {
                    c0291j1.P0();
                    h hVar = c0291j1.m;
                    hVar.getClass();
                    if (!(hVar instanceof C3682d)) {
                    }
                }
                C0945Rl c0945Rl = mVar.a;
                c0945Rl.getClass();
                c0945Rl.a(c0291j1.getReference());
            }
        }
    }

    @Override // com.android.tools.r8.internal.AbstractC3247vi
    public final void j(final D2 d2) {
        final m mVarA = ((o) JM.a(this.g, d2, m.b)).a();
        final p pVar = new p(new C0918Ql(new HashMap()));
        a(new Consumer() { // from class: com.android.tools.r8.ir.optimize.info.n$$ExternalSyntheticLambda0
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                this.f$0.a(pVar, d2, mVarA, (D2) obj);
            }
        }, d2);
        if (((C3877i) this.a.g()).t.b(d2)) {
            Iterator<C0291j1> it = d2.G1().iterator();
            while (it.hasNext()) {
                pVar.b(this.a, it.next().h1(), C3682d.b);
            }
        } else {
            for (C0291j1 c0291j1 : d2.G1()) {
                if (!this.a.t().a(d2, c0291j1).e(this.a.M())) {
                    if (!h && !c0291j1.l1()) {
                        c0291j1.P0();
                        h hVar = c0291j1.m;
                        hVar.getClass();
                        if (!(hVar instanceof C3682d)) {
                            c0291j1.P0();
                            if (!c0291j1.m.E()) {
                                throw new AssertionError();
                            }
                        }
                    }
                    pVar.b(this.a, c0291j1.h1(), C3682d.b);
                } else if (!c0291j1.l1()) {
                    C0421y c0421y = this.a;
                    B2 b2H1 = c0291j1.h1();
                    c0291j1.P0();
                    pVar.b(c0421y, b2H1, c0291j1.m);
                }
            }
        }
        if (!d2.f.f()) {
            for (C0291j1 c0291j12 : d2.l(new KX(new HX() { // from class: com.android.tools.r8.ir.optimize.info.n$$ExternalSyntheticLambda1
                @Override // com.android.tools.r8.internal.HX
                public final boolean apply(Object obj) {
                    return ((C0291j1) obj).p1();
                }
            }))) {
                C0918Ql c0918Ql = pVar.a;
                C3682d c3682d = C3682d.b;
                h hVarA = (h) c0918Ql.a(c0291j12);
                if (hVarA.d()) {
                    hVarA = l.a(hVarA.a());
                } else if (!p.c && !(hVarA instanceof C3682d) && !(hVarA instanceof l)) {
                    throw new AssertionError();
                }
                if (!h && !(hVarA instanceof C3682d) && !(hVarA instanceof l)) {
                    throw new AssertionError();
                }
                r rVar = this.f;
                C0409w2 reference = c0291j12.getReference();
                if (r.b) {
                    rVar.getClass();
                } else if (rVar.a.containsKey(reference)) {
                    throw new AssertionError();
                }
                if (!(hVarA instanceof C3682d)) {
                    rVar.a.put(reference, hVarA);
                }
            }
        }
        pVar.a.b.keySet().removeIf(new Predicate() { // from class: com.android.tools.r8.ir.optimize.info.n$$ExternalSyntheticLambda2
            @Override // java.util.function.Predicate
            public final boolean test(Object obj) {
                return n.a(mVarA, (B2) obj);
            }
        });
        if (pVar.a.b.isEmpty()) {
            return;
        }
        this.g.put(d2, pVar);
    }

    @Override // com.android.tools.r8.internal.AbstractC3247vi
    public final void l(D2 d2) {
        HashSet hashSet = new HashSet();
        final m mVar = new m(new C0945Rl(hashSet));
        V3 v3 = this.b;
        Consumer consumer = new Consumer() { // from class: com.android.tools.r8.ir.optimize.info.n$$ExternalSyntheticLambda3
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                this.f$0.a(mVar, (D2) obj);
            }
        };
        v3.getClass();
        v3.a(PX.b, consumer, d2);
        if (hashSet.isEmpty()) {
            return;
        }
        this.g.put(d2, mVar);
    }

    public final void a(p pVar, D2 d2, m mVar, D2 d22) {
        pVar.a(this.a, ((o) this.g.getOrDefault(d22, p.b)).b());
        if (!d2.isInterface() || d22.isInterface()) {
            return;
        }
        boolean z = h;
        if (!z && !d2.isInterface()) {
            throw new AssertionError();
        }
        if (!z && d22.isInterface()) {
            throw new AssertionError();
        }
        HashSet<B2> hashSet = new HashSet(mVar.a.b);
        new C0945Rl(hashSet).a(d2.G1());
        for (B2 b2 : hashSet) {
            C3877i c3877i = (C3877i) this.a.g();
            if (!C0285j.i) {
                c3877i.c();
            } else {
                c3877i.getClass();
            }
            S4 s4A = c3877i.a(d22, b2.b(), b2.a());
            if (s4A.h()) {
                if (!h && !s4A.k().y()) {
                    throw new AssertionError();
                }
            } else if (s4A.v()) {
                pVar.b(this.a, b2, C3682d.b);
            } else {
                if (!h && !s4A.w()) {
                    throw new AssertionError();
                }
                H0 h0P = s4A.p();
                if (!h0P.a().isInterface() && h0P.a() != d22) {
                    pVar.b(this.a, b2, h0P.D());
                }
            }
        }
    }

    public static boolean a(m mVar, B2 b2) {
        return !mVar.a.b.contains(b2);
    }
}
