package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.B1;
import com.android.tools.r8.graph.C0291j1;
import com.android.tools.r8.graph.C0409w2;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.S4;
import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.AbstractC1308bC;
import com.android.tools.r8.internal.C0756Kt;
import com.android.tools.r8.internal.C0874Ot;
import com.android.tools.r8.internal.C1318bK;
import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.internal.H5;
import com.android.tools.r8.internal.K5;
import com.android.tools.r8.internal.MT;
import com.android.tools.r8.internal.PT;
import com.android.tools.r8.internal.QT;
import com.android.tools.r8.internal.TB;
import com.android.tools.r8.shaking.C3877i;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Predicate;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class M {
    public static final /* synthetic */ boolean c = true;
    public final C0421y a;
    public final B1 b;

    public M(C0421y c0421y) {
        this.a = c0421y;
        this.b = c0421y.a();
    }

    public final void a(final C0874Ot c0874Ot) {
        com.android.tools.r8.graph.H0 h0G;
        PT pt = new PT(new L());
        A5 a5I = c0874Ot.i();
        Iterator<H5> it = c0874Ot.d.iterator();
        while (it.hasNext()) {
            for (AbstractC1076Vw abstractC1076Vw : it.next().k()) {
                if (abstractC1076Vw.W1()) {
                    AbstractC1308bC abstractC1308bCC0 = abstractC1076Vw.c0();
                    if (abstractC1308bCC0.d() != null && !abstractC1076Vw.d().y()) {
                        C0409w2 c0409w2U2 = abstractC1308bCC0.U2();
                        C1318bK c1318bK = this.a.A;
                        c1318bK.getClass();
                        if (!c1318bK.a(abstractC1308bCC0.U2(), abstractC1308bCC0.c) || !this.b.a6.contains(c0409w2U2)) {
                            if (!this.a.o()) {
                                continue;
                            } else {
                                if (!c && !this.a.g().i()) {
                                    throw new AssertionError();
                                }
                                C0421y<C3877i> c0421yV = this.a.V();
                                S4.c<?> cVarO = ((C3877i) c0421yV.g()).b(abstractC1308bCC0.U2(), abstractC1308bCC0.T2()).o();
                                if (cVarO != null && !cVarO.a(a5I, c0421yV).b() && (h0G = abstractC1308bCC0.g(c0421yV, a5I)) != null) {
                                    C0291j1 c0291j1E = h0G.e();
                                    c0291j1E.P0();
                                    com.android.tools.r8.ir.optimize.info.h hVar = c0291j1E.m;
                                    c0421yV.M();
                                    if (hVar.a(abstractC1308bCC0) || !hVar.F() || (abstractC1076Vw.X1() && abstractC1076Vw.d0().V2().i().t().F())) {
                                    }
                                }
                            }
                        }
                        Iterator it2 = abstractC1076Vw.c.iterator();
                        while (true) {
                            if (!it2.hasNext()) {
                                ((List) pt.computeIfAbsent(abstractC1308bCC0, new Function() { // from class: com.android.tools.r8.ir.optimize.M$$ExternalSyntheticLambda1
                                    @Override // java.util.function.Function
                                    public final Object apply(Object obj) {
                                        return M.a((AbstractC1308bC) obj);
                                    }
                                })).add(abstractC1076Vw.d());
                                break;
                            }
                            C3161ul0 c3161ul0 = (C3161ul0) it2.next();
                            if (c3161ul0.l() || !c3161ul0.c.x1() || c3161ul0.c.b().o() != 0) {
                                break;
                            }
                        }
                    }
                }
            }
        }
        if (pt.isEmpty()) {
            return;
        }
        if (!c && c0874Ot.j().x()) {
            throw new AssertionError();
        }
        final HashMap map = new HashMap();
        if (pt.n == null) {
            pt.n = new MT(pt);
        }
        pt.n.stream().filter(new Predicate() { // from class: com.android.tools.r8.ir.optimize.M$$ExternalSyntheticLambda2
            @Override // java.util.function.Predicate
            public final boolean test(Object obj) {
                return M.a((QT) obj);
            }
        }).sorted(new Comparator() { // from class: com.android.tools.r8.ir.optimize.M$$ExternalSyntheticLambda3
            @Override // java.util.Comparator
            public final int compare(Object obj, Object obj2) {
                return Integer.compare(((List) ((QT) obj2).getValue()).size(), ((List) ((QT) obj).getValue()).size());
            }
        }).limit(15L).forEach(new Consumer() { // from class: com.android.tools.r8.ir.optimize.M$$ExternalSyntheticLambda4
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                M.a(c0874Ot, map, (QT) obj);
            }
        });
        if (!map.isEmpty()) {
            Iterator<H5> it3 = c0874Ot.d.iterator();
            while (it3.hasNext()) {
                K5 k5A = it3.next().a(c0874Ot);
                while (k5A.hasNext()) {
                    AbstractC1076Vw next = k5A.next();
                    if (next.W1()) {
                        AbstractC1308bC abstractC1308bCC02 = next.c0();
                        if (map.containsKey(abstractC1308bCC02)) {
                            C3161ul0 c3161ul02 = (C3161ul0) map.get(abstractC1308bCC02);
                            if (!c && c3161ul02 == null) {
                                throw new AssertionError();
                            }
                            abstractC1308bCC02.d().f(c3161ul02);
                            k5A.p();
                        } else {
                            continue;
                        }
                    }
                }
            }
        }
        c0874Ot.a((C0756Kt) null, (C3658a) null);
        c0874Ot.y();
        if (!c && !c0874Ot.b(this.a)) {
            throw new AssertionError();
        }
    }

    public static /* synthetic */ List a(AbstractC1308bC abstractC1308bC) {
        return new ArrayList();
    }

    public static /* synthetic */ boolean a(QT qt) {
        return ((List) qt.getValue()).size() > 1;
    }

    public static void a(C0874Ot c0874Ot, Map map, QT qt) {
        AbstractC1308bC abstractC1308bC = (AbstractC1308bC) qt.getKey();
        C3161ul0 c3161ul0A = c0874Ot.a(abstractC1308bC.a(), abstractC1308bC.d().r());
        TB tbA = TB.a(abstractC1308bC.P2(), abstractC1308bC.U2(), null, c3161ul0A, abstractC1308bC.c, false);
        tbA.b(((C3161ul0) ((List) qt.getValue()).get(0)).c.getPosition());
        if (abstractC1308bC.c.size() > 0) {
            a(c0874Ot, tbA);
        } else {
            K5 k5A = c0874Ot.j().a(c0874Ot);
            while (true) {
                if (!k5A.hasNext()) {
                    break;
                } else if (!k5A.next().l1()) {
                    k5A.previous();
                    break;
                }
            }
            k5A.add(tbA);
        }
        Iterator it = ((List) qt.getValue()).iterator();
        while (it.hasNext()) {
            map.put(((C3161ul0) it.next()).c.c0(), c3161ul0A);
        }
    }

    public static void a(C0874Ot c0874Ot, TB tb) {
        K5 k5A = c0874Ot.j().a(c0874Ot);
        int i = 0;
        while (true) {
            if (!k5A.hasNext()) {
                break;
            }
            AbstractC1076Vw next = k5A.next();
            if (next.d1()) {
                Iterator it = tb.c.iterator();
                while (it.hasNext()) {
                    if (((C3161ul0) it.next()) == next.d()) {
                        i++;
                    }
                }
            }
            if (i == tb.c.size()) {
                if (k5A.hasNext() && k5A.m().l1()) {
                    k5A.a(new Predicate() { // from class: com.android.tools.r8.ir.optimize.M$$ExternalSyntheticLambda0
                        @Override // java.util.function.Predicate
                        public final boolean test(Object obj) {
                            return M.a((AbstractC1076Vw) obj);
                        }
                    });
                }
            }
        }
        k5A.add(tb);
    }

    public static /* synthetic */ boolean a(AbstractC1076Vw abstractC1076Vw) {
        return !abstractC1076Vw.l1();
    }
}
