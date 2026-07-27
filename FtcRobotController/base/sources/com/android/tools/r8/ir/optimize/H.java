package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.AbstractC0275i0;
import com.android.tools.r8.graph.B1;
import com.android.tools.r8.graph.C0291j1;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.C1;
import com.android.tools.r8.graph.I2;
import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.AbstractC1308bC;
import com.android.tools.r8.internal.AbstractC1869gw;
import com.android.tools.r8.internal.AbstractC3424xb0;
import com.android.tools.r8.internal.AbstractC3644zm0;
import com.android.tools.r8.internal.C0874Ot;
import com.android.tools.r8.internal.C1075Vv;
import com.android.tools.r8.internal.C1397c3;
import com.android.tools.r8.internal.C1631ec;
import com.android.tools.r8.internal.C1870gw$$ExternalSyntheticLambda0;
import com.android.tools.r8.internal.C2052iw;
import com.android.tools.r8.internal.C2384mC;
import com.android.tools.r8.internal.C2843rP;
import com.android.tools.r8.internal.C2867rd;
import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.internal.C3200vB;
import com.android.tools.r8.internal.H5;
import com.android.tools.r8.internal.InterfaceC2144jw;
import com.android.tools.r8.internal.JO;
import com.android.tools.r8.internal.K5$$ExternalSyntheticLambda2;
import com.android.tools.r8.internal.Nk0;
import com.android.tools.r8.internal.Y6;
import com.android.tools.r8.shaking.C3877i;
import com.android.tools.r8.shaking.R1;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.Iterator;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.function.Function;
import java.util.function.Predicate;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public class H implements Y {
    public static final /* synthetic */ boolean i = true;
    public final C0421y a;
    public final C3200vB b;
    public final C3200vB.i c;
    public final R1 d;
    public final A5 e;
    public final JO f;
    public final InterfaceC2144jw g;
    public int h;

    public H(C0421y c0421y, InterfaceC2144jw interfaceC2144jw, A5 a5, JO jo, int i2) {
        this.a = c0421y;
        C3200vB c3200vBM = c0421y.M();
        this.b = c3200vBM;
        this.c = c3200vBM.T();
        this.g = interfaceC2144jw;
        this.d = ((C3877i) c0421y.g()).f();
        this.e = a5;
        this.f = jo;
        this.h = i2;
    }

    @Override // com.android.tools.r8.ir.optimize.Y
    public final boolean a() {
        return false;
    }

    @Override // com.android.tools.r8.ir.optimize.Y
    public final C0421y b() {
        return this.a;
    }

    public final boolean c(AbstractC1308bC abstractC1308bC, A5 a5, Optional optional) {
        int iA;
        if (!this.f.a(a5)) {
            AbstractC0275i0 abstractC0275i0V0 = a5.e().V0();
            int i2 = this.c.c;
            if (i2 < 0) {
                i2 = 5;
            }
            int i3 = 0;
            if (this.b.T().d) {
                iA = ((this.b.Z() && abstractC1308bC.d1() && abstractC1308bC.d().z()) ? 1 : 0) + a(abstractC1308bC, a5, optional) + b(a5, abstractC1308bC);
            } else {
                iA = 0;
            }
            int iK = abstractC0275i0V0.k(iA + i2);
            if (iK >= 0) {
                if (iK <= i2) {
                    return true;
                }
                if (this.b.T().d) {
                    int iB = b(abstractC1308bC, a5, optional) + b(a5, abstractC1308bC);
                    if (this.b.Z() && abstractC1308bC.d1() && abstractC1308bC.d().z()) {
                        i3 = 1;
                    }
                    i3 += iB;
                }
                if (iK <= i2 + i3) {
                    return true;
                }
            }
        }
        C0291j1 c0291j1E = a5.e();
        c0291j1E.P0();
        return c0291j1E.m.u().a(abstractC1308bC);
    }

    public static boolean a(AbstractC1308bC abstractC1308bC, A5 a5, AbstractC3644zm0 abstractC3644zm0) {
        if (a5.e().n1()) {
            throw new Nk0("Unexpected attempt to invoke a class initializer (`" + a5.v() + "`)");
        }
        if (!a5.e().j1()) {
            abstractC3644zm0.g();
            return true;
        }
        int size = ((ArrayList) abstractC1308bC.K2()).size() - Y6.a(abstractC1308bC.X1());
        int iA0 = a5.getReference().A0();
        if (size == iA0) {
            return false;
        }
        abstractC3644zm0.a(size, iA0);
        return true;
    }

    public static int b(A5 a5, AbstractC1308bC abstractC1308bC) {
        BitSet bitSetS = a5.D().s();
        int i2 = 0;
        if (bitSetS == null) {
            return 0;
        }
        for (int iA = Y6.a(abstractC1308bC.X1()); iA < abstractC1308bC.c.size(); iA++) {
            C3161ul0 c3161ul0B = abstractC1308bC.b(iA);
            if (bitSetS.get(iA) && c3161ul0B.t().I() && c3161ul0B.M()) {
                i2 += 4;
            }
        }
        return i2;
    }

    public final int b(AbstractC1308bC abstractC1308bC, A5 a5, Optional optional) {
        final C1 c1;
        int i2 = 0;
        if (!H$$ExternalSyntheticBackport0.m(optional) && !abstractC1308bC.c.isEmpty() && a5.e().V0().D0()) {
            final C0874Ot c0874OtA = ((C2052iw) optional.get()).a(a5, abstractC1308bC);
            Objects.requireNonNull(c0874OtA);
            Iterable<C1397c3> iterable = new Iterable() { // from class: com.android.tools.r8.ir.optimize.H$$ExternalSyntheticLambda3
                @Override // java.lang.Iterable
                public final Iterator iterator() {
                    return c0874OtA.b();
                }
            };
            B1 b1A = this.a.a();
            for (C1397c3 c1397c3 : iterable) {
                C3161ul0 c3161ul0D = c1397c3.d();
                for (AbstractC1076Vw abstractC1076Vw : c3161ul0D.b0()) {
                    if (abstractC1076Vw.v1()) {
                        if (abstractC1308bC.b(c1397c3.b(true)).t().a(abstractC1076Vw.B().i.b(this.a), this.a)) {
                            i2 += 2;
                        }
                    } else {
                        I2 i2A = a5.a(c1397c3.b(true));
                        b1A.getClass();
                        if (i2A.T0()) {
                            c1 = new C1(b1A.c(i2A), b1A.g(i2A));
                        } else {
                            c1 = b1A.S5.containsValue(i2A) ? new C1(b1A.g(i2A), b1A.c(i2A)) : null;
                        }
                        if (c1 != null) {
                            C3161ul0 c3161ul0I = abstractC1308bC.b(c1397c3.b(true)).i();
                            if (abstractC1076Vw.a(c1.a) && c3161ul0I.c(new Predicate() { // from class: com.android.tools.r8.ir.optimize.H$$ExternalSyntheticLambda4
                                @Override // java.util.function.Predicate
                                public final boolean test(Object obj) {
                                    return H.b(c1, (AbstractC1076Vw) obj);
                                }
                            })) {
                                i2 = (c3161ul0I.U() == 1 && c3161ul0D.U() == 1) ? i2 + 8 : i2 + 4;
                            }
                        }
                    }
                }
            }
        }
        return i2;
    }

    /* JADX WARN: Removed duplicated region for block: B:67:0x0145  */
    /* JADX WARN: Removed duplicated region for block: B:74:0x016d  */
    @Override // com.android.tools.r8.ir.optimize.Y
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public final boolean a(com.android.tools.r8.internal.C0874Ot r7, com.android.tools.r8.graph.S4.c r8, com.android.tools.r8.graph.A5 r9, com.android.tools.r8.internal.AbstractC3644zm0 r10) {
        /*
            Method dump skipped, instruction units count: 430
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.android.tools.r8.ir.optimize.H.a(com.android.tools.r8.internal.Ot, com.android.tools.r8.graph.S4$c, com.android.tools.r8.graph.A5, com.android.tools.r8.internal.zm0):boolean");
    }

    public static boolean b(C1 c1, AbstractC1076Vw abstractC1076Vw) {
        return abstractC1076Vw.a(c1.b);
    }

    public final boolean a(C0874Ot c0874Ot, AbstractC1308bC abstractC1308bC, C0874Ot c0874Ot2, AbstractC3644zm0 abstractC3644zm0) {
        if (!c0874Ot.i.b(42) || !c0874Ot2.i.b(42)) {
            return false;
        }
        Set setC = AbstractC3424xb0.c();
        Set setC2 = AbstractC3424xb0.c();
        AbstractC1869gw.a(c0874Ot, setC, setC2);
        if (setC.isEmpty() && setC2.isEmpty()) {
            return false;
        }
        Iterator it = c0874Ot2.b((Predicate) new C1870gw$$ExternalSyntheticLambda0()).iterator();
        while (it.hasNext()) {
            C3161ul0 c3161ul0I = ((C3161ul0) ((C2843rP) it.next()).c.get(0)).i();
            if (c3161ul0I.c(new K5$$ExternalSyntheticLambda2())) {
                c3161ul0I = ((C3161ul0) abstractC1308bC.c.get(c3161ul0I.c.t().b(true))).i();
            }
            AbstractC1869gw.a(c3161ul0I, setC, setC2);
        }
        int size = setC2.size() + setC.size();
        int i2 = this.c.g;
        if (size <= i2) {
            return false;
        }
        abstractC3644zm0.d(size, i2);
        return true;
    }

    public final int a(AbstractC1308bC abstractC1308bC, A5 a5, Optional optional) {
        final C1 c1;
        int i2 = 0;
        if (!H$$ExternalSyntheticBackport0.m(optional) && !abstractC1308bC.c.isEmpty() && a5.e().V0().D0()) {
            for (int iA = Y6.a(abstractC1308bC.X1()); iA < abstractC1308bC.c.size(); iA++) {
                C3161ul0 c3161ul0I = abstractC1308bC.b(iA).i();
                if (c3161ul0I.t().I()) {
                    i2 += 2;
                }
                I2 i2A = a5.a(iA);
                B1 b1A = this.a.a();
                b1A.getClass();
                if (i2A.T0()) {
                    c1 = new C1(b1A.c(i2A), b1A.g(i2A));
                } else {
                    c1 = b1A.S5.containsValue(i2A) ? new C1(b1A.g(i2A), b1A.c(i2A)) : null;
                }
                if (c1 != null && c3161ul0I.c(new Predicate() { // from class: com.android.tools.r8.ir.optimize.H$$ExternalSyntheticLambda2
                    @Override // java.util.function.Predicate
                    public final boolean test(Object obj) {
                        return H.a(c1, (AbstractC1076Vw) obj);
                    }
                })) {
                    i2 += 8;
                }
            }
        }
        return i2;
    }

    @Override // com.android.tools.r8.ir.optimize.Y
    public final A5 a(A5 a5, AbstractC1308bC abstractC1308bC) {
        return com.android.tools.r8.graph.H0.a(abstractC1308bC.g(this.a, a5));
    }

    /* JADX WARN: Removed duplicated region for block: B:20:0x0084  */
    @Override // com.android.tools.r8.ir.optimize.Y
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public com.android.tools.r8.ir.optimize.S a(com.android.tools.r8.internal.C0874Ot r19, com.android.tools.r8.internal.AbstractC1308bC r20, com.android.tools.r8.graph.S4.c r21, com.android.tools.r8.graph.A5 r22, com.android.tools.r8.graph.A5 r23, com.android.tools.r8.internal.C1631ec r24, com.android.tools.r8.internal.C2052iw r25, com.android.tools.r8.internal.AbstractC3644zm0 r26) {
        /*
            Method dump skipped, instruction units count: 1036
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.android.tools.r8.ir.optimize.H.a(com.android.tools.r8.internal.Ot, com.android.tools.r8.internal.bC, com.android.tools.r8.graph.S4$c, com.android.tools.r8.graph.A5, com.android.tools.r8.graph.A5, com.android.tools.r8.internal.ec, com.android.tools.r8.internal.iw, com.android.tools.r8.internal.zm0):com.android.tools.r8.ir.optimize.S");
    }

    public final boolean a(C2384mC c2384mC, final A5 a5, final A5 a52, C1631ec c1631ec) {
        if (((C3877i) this.a.g()).c(a5.s(), a52.s())) {
            return true;
        }
        return (!a5.e().z0() && ((Boolean) this.a.b(new Function() { // from class: com.android.tools.r8.ir.optimize.H$$ExternalSyntheticLambda1
            @Override // java.util.function.Function
            public final Object apply(Object obj) {
                return Boolean.valueOf(((C1075Vv) obj).a(a52.a(), a5));
            }
        })).booleanValue()) || c1631ec.a(a52.s(), c2384mC) || !a52.a().a(this.a, a5) || this.a.P().g.contains(a52.getReference());
    }

    @Override // com.android.tools.r8.ir.optimize.Y
    public final boolean a(Q q, AbstractC3644zm0 abstractC3644zm0) {
        if (q.c == U.b) {
            return true;
        }
        boolean z = this.h > 0;
        if (!z) {
            abstractC3644zm0.n();
        }
        return z;
    }

    @Override // com.android.tools.r8.ir.optimize.Y
    public final boolean a(Q q, C0874Ot c0874Ot, C0874Ot c0874Ot2, AbstractC1308bC abstractC1308bC, H5 h5, AbstractC3644zm0 abstractC3644zm0) {
        if (q.c == U.b) {
            return false;
        }
        int iB = W.b(c0874Ot2);
        if (this.h < W.b(c0874Ot2)) {
            abstractC3644zm0.c(iB, this.h);
        } else if (!a(c0874Ot, abstractC1308bC, c0874Ot2, abstractC3644zm0)) {
            if (!h5.x()) {
                return false;
            }
            Iterator<H5> it = c0874Ot2.d.iterator();
            int i2 = 0;
            while (it.hasNext()) {
                Iterator<AbstractC1076Vw> it2 = it.next().k().iterator();
                int i3 = 0;
                while (it2.hasNext()) {
                    if (it2.next().i()) {
                        i3++;
                    }
                }
                i2 += i3;
            }
            int size = h5.e.size() * i2;
            int i4 = this.c.h;
            if (size < i4) {
                return false;
            }
            abstractC3644zm0.b(size, i4);
        }
        return true;
    }

    @Override // com.android.tools.r8.ir.optimize.Y
    public final void a(C0874Ot c0874Ot) {
        this.h -= W.b(c0874Ot);
    }

    @Override // com.android.tools.r8.ir.optimize.Y
    public final C2867rd a(AbstractC1308bC abstractC1308bC, C2867rd c2867rd) {
        return c2867rd;
    }

    public static boolean a(C1 c1, AbstractC1076Vw abstractC1076Vw) {
        return abstractC1076Vw.a(c1.b);
    }
}
