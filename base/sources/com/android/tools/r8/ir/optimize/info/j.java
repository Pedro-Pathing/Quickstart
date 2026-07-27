package com.android.tools.r8.ir.optimize.info;

import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.B1;
import com.android.tools.r8.graph.C0291j1;
import com.android.tools.r8.graph.C0409w2;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.D2;
import com.android.tools.r8.graph.I2;
import com.android.tools.r8.internal.A4;
import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.AbstractC2215kc;
import com.android.tools.r8.internal.AbstractC3250vj0;
import com.android.tools.r8.internal.AbstractC3424xb0;
import com.android.tools.r8.internal.AbstractC3650zs;
import com.android.tools.r8.internal.C0580Fb;
import com.android.tools.r8.internal.C0874Ot;
import com.android.tools.r8.internal.C1020Tt;
import com.android.tools.r8.internal.C1397c3;
import com.android.tools.r8.internal.C2384mC;
import com.android.tools.r8.internal.C2435mm;
import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.internal.C3200vB;
import com.android.tools.r8.internal.C3395xC;
import com.android.tools.r8.internal.El0$$ExternalSyntheticLambda3;
import com.android.tools.r8.internal.EnumC2621ou;
import com.android.tools.r8.internal.H5;
import com.android.tools.r8.internal.HA;
import com.android.tools.r8.internal.HC;
import com.android.tools.r8.internal.InterfaceC1102Ww;
import com.android.tools.r8.internal.J5;
import com.android.tools.r8.internal.OC;
import com.android.tools.r8.internal.PX;
import com.android.tools.r8.kotlin.C3726d;
import com.android.tools.r8.shaking.C3877i;
import java.util.ArrayDeque;
import java.util.Iterator;
import java.util.Set;
import java.util.function.BiFunction;
import java.util.function.Predicate;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class j {
    public static final /* synthetic */ boolean e = true;
    public final C0421y a;
    public final C0580Fb b;
    public final B1 c;
    public final C3200vB d;

    public j(C0421y c0421y, C1020Tt c1020Tt) {
        this.a = c0421y;
        this.b = !c0421y.M().Z0 ? new C0580Fb(c0421y, c1020Tt) : null;
        this.c = c0421y.a();
        this.d = c0421y.M();
    }

    public static boolean a(C3161ul0 c3161ul0, C3161ul0 c3161ul02) {
        return (c3161ul0.l() && c3161ul02.A()) || c3161ul0.a(A4.a, PX.c) == c3161ul02;
    }

    public static /* synthetic */ boolean b(D2 d2) {
        return true;
    }

    public final boolean a(final C0874Ot c0874Ot, final C3161ul0 c3161ul0) {
        i iVar;
        BiFunction biFunction = new BiFunction() { // from class: com.android.tools.r8.ir.optimize.info.j$$ExternalSyntheticLambda2
            @Override // java.util.function.BiFunction
            public final Object apply(Object obj, Object obj2) {
                return this.f$0.a(c3161ul0, c0874Ot, (AbstractC1076Vw) obj, (InterfaceC1102Ww) obj2);
            }
        };
        int iA = c0874Ot.A();
        try {
            ArrayDeque arrayDeque = new ArrayDeque();
            H5 h5J = c0874Ot.j();
            arrayDeque.add(h5J);
            h5J.b(iA);
            while (!arrayDeque.isEmpty()) {
                H5 h5 = (H5) arrayDeque.poll();
                if (!e && !h5.a(iA)) {
                    throw new AssertionError();
                }
                i iVar2 = i.e;
                J5 j5H = h5.H();
                while (true) {
                    iVar = i.e;
                    if (iVar2 != iVar || !j5H.hasNext()) {
                        break;
                    }
                    iVar2 = (i) biFunction.apply(j5H.next(), j5H);
                }
                if (iVar2 != i.d) {
                    if (iVar2 != i.b) {
                        if (iVar2 == i.c) {
                            boolean z = e;
                            if (!z && h5.n().isEmpty()) {
                                throw new AssertionError();
                            }
                            AbstractC1076Vw last = h5.k().getLast();
                            if (!z && !last.M1()) {
                                throw new AssertionError();
                            }
                            H5 h5B = last.T().b(0);
                            if (!h5B.a(iA)) {
                                arrayDeque.add(h5B);
                                h5B.b(iA);
                            }
                        } else {
                            boolean z2 = e;
                            if (!z2 && iVar2 != iVar) {
                                throw new AssertionError();
                            }
                            if (h5.n().isEmpty()) {
                                AbstractC1076Vw last2 = h5.k().getLast();
                                if (!z2 && !last2.v2() && !last2.C2()) {
                                    throw new AssertionError();
                                }
                            } else {
                                for (H5 h52 : h5.t()) {
                                    if (!h52.a(iA)) {
                                        arrayDeque.add(h52);
                                        h52.b(iA);
                                    }
                                }
                            }
                        }
                    }
                }
                return false;
            }
            c0874Ot.a(iA);
            return true;
        } finally {
            c0874Ot.a(iA);
        }
    }

    public final i a(C3161ul0 c3161ul0, C0874Ot c0874Ot, AbstractC1076Vw abstractC1076Vw, InterfaceC1102Ww interfaceC1102Ww) {
        AbstractC1076Vw abstractC1076VwM;
        H5 h5B = abstractC1076Vw.b();
        if (!h5B.x() && a(abstractC1076Vw, c3161ul0)) {
            return i.c;
        }
        if (abstractC1076Vw.a2()) {
            C2384mC c2384mCG0 = abstractC1076Vw.g0();
            C0421y c0421y = this.a;
            if (!c0421y.M().n1.a) {
                C3726d c3726d = c0421y.a().J4.c;
                AbstractC3650zs abstractC3650zsA = c0421y.A();
                C0409w2 c0409w2U2 = c2384mCG0.U2();
                abstractC3650zsA.getClass();
                C0409w2 c0409w2A = abstractC3650zsA.a(AbstractC3650zs.g(), c0409w2U2);
                if ((c0409w2A.d(c3726d.c) || c0409w2A.d(c3726d.d)) && c2384mCG0.b(0) == c3161ul0 && c0409w2A.w0().D0().startsWith("kotlin")) {
                    return i.b;
                }
            }
            C0421y c0421y2 = this.a;
            if (!c0421y2.M().n1.a) {
                C3726d c3726d2 = c0421y2.a().J4.c;
                AbstractC3650zs abstractC3650zsA2 = c0421y2.A();
                C0409w2 c0409w2U22 = c2384mCG0.U2();
                abstractC3650zsA2.getClass();
                C0409w2 c0409w2A2 = abstractC3650zsA2.a(AbstractC3650zs.g(), c0409w2U22);
                if ((c0409w2A2.d(c3726d2.a) || c0409w2A2.d(c3726d2.b)) && c0409w2A2.w0().D0().startsWith("kotlin")) {
                    Iterator<H5> it = h5B.s().iterator();
                    while (it.hasNext()) {
                        if (a(it.next().h(), c3161ul0)) {
                            return i.b;
                        }
                    }
                    return i.e;
                }
            }
        }
        B1 b1A = this.a.a();
        if (abstractC1076Vw.o2() && abstractC1076Vw.u0().i == b1A.v3 && (abstractC1076VwM = interfaceC1102Ww.m()) != null && abstractC1076VwM.U1() && abstractC1076VwM.a0().U2() == b1A.F4.a) {
            interfaceC1102Ww.next();
            return i.e;
        }
        if (abstractC1076Vw.a(this.a, c0874Ot.i(), c3161ul0)) {
            if (!h5B.x()) {
                return i.b;
            }
        } else if (abstractC1076Vw.b(this.a, c0874Ot.i())) {
            if (abstractC1076Vw.B1() && !abstractC1076Vw.a(this.a, c0874Ot.i())) {
                return i.e;
            }
            return i.d;
        }
        return i.e;
    }

    public static boolean a(AbstractC1076Vw abstractC1076Vw, C3161ul0 c3161ul0) {
        if (abstractC1076Vw.M1() && abstractC1076Vw.T().P2() && ((C3161ul0) abstractC1076Vw.c.get(0)).equals(c3161ul0)) {
            return abstractC1076Vw.T().j == EnumC2621ou.b || abstractC1076Vw.T().j == EnumC2621ou.g;
        }
        return false;
    }

    public final void a(y yVar, C0291j1 c0291j1, C0874Ot c0874Ot) {
        boolean z = e;
        if (!z && c0291j1.g.M()) {
            throw new AssertionError();
        }
        if (this.d.H && !((C3877i) this.a.g()).w.containsKey(c0291j1.getReference())) {
            A5 a5I = c0874Ot.i();
            if (c0291j1.n1()) {
                int iA = AbstractC2215kc.a(this.a, c0874Ot);
                if (iA == 3) {
                    yVar.b(c0291j1);
                    yVar.e(c0291j1);
                    return;
                }
                if (iA != 1) {
                    yVar.e(c0291j1);
                    return;
                } else {
                    if (z || this.d.Z0 || this.a.y().c()) {
                        return;
                    }
                    this.a.y().a(a5I.a(), new Predicate() { // from class: com.android.tools.r8.ir.optimize.info.j$$ExternalSyntheticLambda3
                        @Override // java.util.function.Predicate
                        public final boolean test(Object obj) {
                            return this.f$0.a((D2) obj);
                        }
                    }, new Predicate() { // from class: com.android.tools.r8.ir.optimize.info.j$$ExternalSyntheticLambda4
                        @Override // java.util.function.Predicate
                        public final boolean test(Object obj) {
                            return j.b((D2) obj);
                        }
                    });
                    return;
                }
            }
            if (c0291j1.A1()) {
                return;
            }
            if (c0291j1.r1()) {
                D2 d2A = a5I.a();
                if (!d2A.isInterface()) {
                    B1 b1A = this.a.a();
                    C0291j1 c0291j1S = ((C3877i) this.a.g()).d(d2A, this.a.a().p4.g).s();
                    if (c0291j1S != null && c0291j1S.getReference() != b1A.B4.l && c0291j1S.getReference() != b1A.p4.g) {
                        return;
                    }
                }
            }
            for (AbstractC1076Vw abstractC1076Vw : c0874Ot.s()) {
                if (abstractC1076Vw.a(this.a.a()) && abstractC1076Vw.a0().V2().i().c(new El0$$ExternalSyntheticLambda3())) {
                    if (abstractC1076Vw.c(this.a, a5I)) {
                        return;
                    }
                } else if (abstractC1076Vw.b(this.a, a5I)) {
                    return;
                }
            }
            yVar.b(c0291j1);
        }
    }

    public final boolean a(D2 d2) {
        C3877i c3877i = (C3877i) this.a.g();
        I2 type = d2.getType();
        if (!C3877i.J) {
            c3877i.getClass();
            if (!type.M0()) {
                throw new AssertionError();
            }
        }
        D2 d2B = D2.b(c3877i.d(type));
        if (d2B != null) {
            C3395xC c3395xCI1 = d2B.I1();
            HC hcA = OC.a(c3395xCI1.b.iterator(), c3395xCI1.c);
            while (hcA.b.hasNext()) {
                A5 a5 = (A5) hcA.a(hcA.b.next());
                if (a5.e().r1() && c3877i.a(a5)) {
                    return true;
                }
            }
        }
        return false;
    }

    public final boolean a(C0874Ot c0874Ot, C3161ul0 c3161ul0, C2435mm c2435mm, Set set) {
        if (!e && !c3161ul0.t().I()) {
            throw new AssertionError();
        }
        if (c3161ul0.l) {
            return true;
        }
        Set setC = AbstractC3424xb0.c();
        for (final AbstractC1076Vw abstractC1076Vw : c3161ul0.c()) {
            if (abstractC1076Vw.t1()) {
                if (!e && abstractC1076Vw.b().x() && abstractC1076Vw.b().k().stream().filter(new Predicate() { // from class: com.android.tools.r8.ir.optimize.info.j$$ExternalSyntheticLambda1
                    @Override // java.util.function.Predicate
                    public final boolean test(Object obj) {
                        return j.a(abstractC1076Vw, (AbstractC1076Vw) obj);
                    }
                }).findFirst().get() != abstractC1076Vw) {
                    throw new AssertionError();
                }
                setC.add(abstractC1076Vw.b());
            } else if (abstractC1076Vw.a(this.a, c0874Ot.i(), c3161ul0)) {
                if (abstractC1076Vw.b().x()) {
                    setC.addAll(abstractC1076Vw.b().n());
                } else {
                    setC.add(abstractC1076Vw.b());
                }
            } else if (abstractC1076Vw.M1() && abstractC1076Vw.T().P2() && (abstractC1076Vw.T().j == EnumC2621ou.b || abstractC1076Vw.T().j == EnumC2621ou.g)) {
                setC.add(abstractC1076Vw.T().S2());
            }
        }
        if (setC.isEmpty()) {
            return false;
        }
        Iterator it = set.iterator();
        while (it.hasNext()) {
            H5 h5 = (H5) it.next();
            Iterator it2 = setC.iterator();
            while (true) {
                if (it2.hasNext()) {
                    if (c2435mm.a(h5, (H5) it2.next())) {
                        break;
                    }
                } else {
                    Set setC2 = AbstractC3424xb0.c();
                    ArrayDeque arrayDeque = new ArrayDeque(h5.s());
                    while (!arrayDeque.isEmpty()) {
                        H5 h52 = (H5) arrayDeque.poll();
                        if (h52 != c0874Ot.j()) {
                            if (!setC2.add(h52)) {
                                if (arrayDeque.isEmpty()) {
                                }
                            } else {
                                Iterator it3 = setC.iterator();
                                while (true) {
                                    if (it3.hasNext()) {
                                        if (c2435mm.a(h52, (H5) it3.next())) {
                                            break;
                                        }
                                    } else {
                                        arrayDeque.addAll(h52.s());
                                        break;
                                    }
                                }
                            }
                        }
                        return false;
                    }
                    if (!e && !arrayDeque.isEmpty()) {
                        throw new AssertionError();
                    }
                }
            }
        }
        return true;
    }

    public static /* synthetic */ boolean a(AbstractC1076Vw abstractC1076Vw, AbstractC1076Vw abstractC1076Vw2) {
        return abstractC1076Vw2 == abstractC1076Vw || abstractC1076Vw2.i();
    }

    public static boolean a(C1397c3 c1397c3) {
        AbstractC3250vj0 abstractC3250vj0A = c1397c3.a();
        abstractC3250vj0A.getClass();
        return (abstractC3250vj0A instanceof HA) && c1397c3.d().b(new Predicate() { // from class: com.android.tools.r8.ir.optimize.info.j$$ExternalSyntheticLambda0
            @Override // java.util.function.Predicate
            public final boolean test(Object obj) {
                return ((AbstractC1076Vw) obj).k1();
            }
        });
    }
}
