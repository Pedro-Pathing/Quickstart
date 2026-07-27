package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.ClassFileConsumer;
import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.AbstractC0275i0;
import com.android.tools.r8.graph.B1;
import com.android.tools.r8.graph.C0291j1;
import com.android.tools.r8.graph.C0299k2;
import com.android.tools.r8.graph.C0307l;
import com.android.tools.r8.graph.C0409w2;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.E2;
import com.android.tools.r8.graph.E4;
import com.android.tools.r8.graph.F2;
import com.android.tools.r8.graph.I2;
import com.android.tools.r8.internal.AbstractC0610Gd;
import com.android.tools.r8.internal.AbstractC0695Iu;
import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.AbstractC3415xU;
import com.android.tools.r8.internal.AbstractC3581z9;
import com.android.tools.r8.internal.B8;
import com.android.tools.r8.internal.C0599Fu;
import com.android.tools.r8.internal.C0607Ga;
import com.android.tools.r8.internal.C0611Ge;
import com.android.tools.r8.internal.C0874Ot;
import com.android.tools.r8.internal.C1027Ua;
import com.android.tools.r8.internal.C1134Ya;
import com.android.tools.r8.internal.C1164Za;
import com.android.tools.r8.internal.C1304b9;
import com.android.tools.r8.internal.C1456cg;
import com.android.tools.r8.internal.C1982i9;
import com.android.tools.r8.internal.C2384mC;
import com.android.tools.r8.internal.C2929sC;
import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.internal.C3306wL;
import com.android.tools.r8.internal.C3328wa;
import com.android.tools.r8.internal.C3384x50;
import com.android.tools.r8.internal.EX;
import com.android.tools.r8.internal.Ec0;
import com.android.tools.r8.internal.El0$$ExternalSyntheticLambda0;
import com.android.tools.r8.internal.G9;
import com.android.tools.r8.internal.Hl0;
import com.android.tools.r8.internal.InterfaceC0710Jd;
import com.android.tools.r8.internal.InterfaceC1160Yw;
import com.android.tools.r8.internal.InterfaceC1376br;
import com.android.tools.r8.internal.JO;
import com.android.tools.r8.internal.K9;
import com.android.tools.r8.internal.LN;
import com.android.tools.r8.internal.P9;
import com.android.tools.r8.internal.S40;
import com.android.tools.r8.internal.W9;
import com.android.tools.r8.internal.X9;
import com.android.tools.r8.origin.Origin;
import com.android.tools.r8.shaking.C3877i;
import com.android.tools.r8.synthesis.S;
import com.sun.tools.javac.jvm.ByteCodes;
import java.util.ArrayList;
import java.util.Collection;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class v0 extends AbstractC0610Gd {
    public static final /* synthetic */ boolean g = true;
    public final com.android.tools.r8.androidapi.a e;
    public final C0299k2 f;

    public v0(C0421y c0421y) {
        super(c0421y);
        this.e = c0421y.T;
        this.f = c0421y.a().P5;
    }

    public final void a(A5 a5, I2 i2, String str) {
        String str2;
        C3877i c3877i = (C3877i) this.a.g();
        if (!c3877i.y.contains(this.f.a)) {
            if (!c3877i.y.contains(this.f.b)) {
                return;
            }
        }
        C3384x50 c3384x50O = this.a.O();
        Origin origin = a5.getOrigin();
        if (i2 == null) {
            str2 = "";
        } else {
            str2 = " of type " + i2.H0();
        }
        c3384x50O.info(new ServiceLoaderRewriterDiagnostic(origin, "Could not inline ServiceLoader.load" + str2 + ": " + str));
    }

    @Override // com.android.tools.r8.internal.AbstractC0610Gd
    public final String b() {
        return "ServiceLoaderRewriter";
    }

    @Override // com.android.tools.r8.internal.AbstractC0610Gd
    public final InterfaceC0710Jd b(final C0874Ot c0874Ot, final JO jo, final C0611Ge c0611Ge) {
        InterfaceC1160Yw interfaceC1160YwR = c0874Ot.r();
        IdentityHashMap identityHashMap = new IdentityHashMap();
        while (interfaceC1160YwR.hasNext()) {
            AbstractC1076Vw next = interfaceC1160YwR.next();
            if (next.a2()) {
                C2384mC c2384mCG0 = next.g0();
                C0409w2 c0409w2U2 = c2384mCG0.U2();
                if (this.f.a(c0409w2U2)) {
                    C3161ul0 c3161ul0I = c2384mCG0.M2().i();
                    if (c3161ul0I.c(new El0$$ExternalSyntheticLambda0())) {
                        C1456cg c1456cgD = c3161ul0I.m().D();
                        if (c0409w2U2.b(this.f.b)) {
                            a(c0874Ot.i(), c1456cgD.K2(), "Inlining is only supported for `java.util.ServiceLoader.load(java.lang.Class, java.lang.ClassLoader)`");
                        } else {
                            C3161ul0 c3161ul0D = c2384mCG0.d();
                            if (!c3161ul0D.B() || c3161ul0D.A()) {
                                a(c0874Ot.i(), c1456cgD.K2(), "The returned ServiceLoader instance must only be used in a call to `java.util.Iterator java.lang.ServiceLoader.iterator()`");
                            } else {
                                C2929sC c2929sCI0 = c3161ul0D.Z().i0();
                                if (c2929sCI0 == null || c2929sCI0.U2().b(this.f.d)) {
                                    a(c0874Ot.i(), c1456cgD.K2(), "The returned ServiceLoader instance must only be used in a call to `java.util.Iterator java.lang.ServiceLoader.iterator()`, but found other usages");
                                } else if (((C3877i) a().g()).a((F2) c1456cgD.L2())) {
                                    a(c0874Ot.i(), c1456cgD.K2(), "The service loader type is kept");
                                } else {
                                    C0307l c0307lI = this.a.i();
                                    if (c0307lI.a().contains(c1456cgD.L2())) {
                                        if (c0307lI.a(a(), c1456cgD.L2())) {
                                            a(c0874Ot.i(), c1456cgD.K2(), "The service loader type has implementations in a feature split");
                                        } else {
                                            C3161ul0 c3161ul0I2 = c2384mCG0.N2().i();
                                            if (c3161ul0I2.l()) {
                                                a(c0874Ot.i(), c1456cgD.K2(), "The java.lang.ClassLoader argument must be defined locally as null or " + c1456cgD.K2() + ".class.getClassLoader()");
                                            } else {
                                                C2929sC c2929sCI02 = c3161ul0I2.m().i0();
                                                if (c3161ul0I2.t().E() || (c2929sCI02 != null && ((ArrayList) c2929sCI02.K2()).size() == 1 && c2929sCI02.V2().i().G() && c2929sCI02.V2().i().m().D().L2().a(c1456cgD.L2()))) {
                                                    AbstractC0695Iu<I2> abstractC0695IuA = c0307lI.a(c1456cgD.L2());
                                                    final ArrayList arrayList = new ArrayList(abstractC0695IuA.size());
                                                    boolean z = false;
                                                    for (I2 i2 : abstractC0695IuA) {
                                                        com.android.tools.r8.graph.E0 e0D = this.a.d(i2);
                                                        if (e0D == null) {
                                                            a(c0874Ot.i(), c1456cgD.K2(), "Unable to find definition for service implementation " + i2.H0());
                                                            z = true;
                                                        }
                                                        arrayList.add(e0D);
                                                    }
                                                    if (!z) {
                                                        new u0(c0874Ot, interfaceC1160YwR, c2384mCG0).a(c2929sCI02, ((C0291j1) identityHashMap.computeIfAbsent(c1456cgD.L2(), new Function() { // from class: com.android.tools.r8.ir.optimize.v0$$ExternalSyntheticLambda2
                                                            @Override // java.util.function.Function
                                                            public final Object apply(Object obj) {
                                                                return this.f$0.a(arrayList, jo, c0611Ge, c0874Ot, (I2) obj);
                                                            }
                                                        })).getReference());
                                                    }
                                                } else {
                                                    a(c0874Ot.i(), c1456cgD.K2(), "The java.lang.ClassLoader argument must be defined locally as null or " + c1456cgD.K2() + ".class.getClassLoader()");
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    } else {
                        a(c0874Ot.i(), (I2) null, "The service loader type could not be determined");
                    }
                }
            }
        }
        if (g || c0874Ot.b(this.a)) {
            return identityHashMap.isEmpty() ? InterfaceC0710Jd.a : InterfaceC0710Jd.b;
        }
        throw new AssertionError();
    }

    @Override // com.android.tools.r8.internal.AbstractC0610Gd
    public final boolean a(C0874Ot c0874Ot, JO jo) {
        return this.a.g().i() && jo.f() && this.c.J && c0874Ot.i.b(12) && c0874Ot.i.b(38) && c0874Ot.i.b(40);
    }

    public final void a(E2 e2, final List list, final I2 i2, com.android.tools.r8.synthesis.N n) {
        n.h = E4.b(4105, false);
        n.e = e2;
        n.l = this.a.U;
        com.android.tools.r8.androidapi.a aVar = this.e;
        List listA = C3306wL.a((Collection) list, new Function() { // from class: com.android.tools.r8.ir.optimize.v0$$ExternalSyntheticLambda3
            @Override // java.util.function.Function
            public final Object apply(Object obj) {
                return ((com.android.tools.r8.graph.E0) obj).e;
            }
        });
        aVar.getClass();
        int i = com.android.tools.r8.androidapi.f.a;
        com.android.tools.r8.androidapi.h hVar = com.android.tools.r8.androidapi.h.b;
        n.m = aVar.a(listA);
        n.g = new com.android.tools.r8.synthesis.M() { // from class: com.android.tools.r8.ir.optimize.v0$$ExternalSyntheticLambda4
            @Override // com.android.tools.r8.synthesis.M
            public final AbstractC0275i0 a(C0409w2 c0409w2) {
                return this.f$0.a(i2, list, c0409w2);
            }
        };
    }

    public final C0291j1 a(List list, JO jo, C0611Ge c0611Ge, C0874Ot c0874Ot, I2 i2) {
        C0291j1 c0291j1A = a(i2, list, jo, c0611Ge);
        if (this.a.M().j instanceof ClassFileConsumer) {
            c0291j1A.b(c0874Ot.i().e().U0());
        }
        return c0291j1A;
    }

    public final C0291j1 a(final I2 i2, final List list, JO jo, C0611Ge c0611Ge) {
        final E2 e2A = this.a.a().a(this.a.a().D5, new I2[0]);
        A5 a5B = this.a.a.g().b(new com.android.tools.r8.synthesis.I() { // from class: com.android.tools.r8.ir.optimize.v0$$ExternalSyntheticLambda0
            @Override // com.android.tools.r8.synthesis.I
            public final S.b a(com.android.tools.r8.synthesis.S s) {
                return s.J;
            }
        }, c0611Ge.a(), this.a, new Consumer() { // from class: com.android.tools.r8.ir.optimize.v0$$ExternalSyntheticLambda1
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                this.f$0.a(e2A, list, i2, (com.android.tools.r8.synthesis.N) obj);
            }
        });
        jo.b(a5B);
        jo.d().c(a5B, c0611Ge.c);
        return a5B.e();
    }

    public final AbstractC0275i0 a(I2 i2, List list, C0409w2 c0409w2) {
        B1 b1A = this.a.a();
        C0599Fu c0599FuG = AbstractC0695Iu.g();
        K9 k9 = new K9();
        K9 k92 = new K9();
        AbstractC3581z9[] abstractC3581z9Arr = {k9, new C1304b9(list.size(), Hl0.c), new X9(b1A.a(1, i2))};
        AbstractC3415xU.a(3, abstractC3581z9Arr);
        c0599FuG.a(3, abstractC3581z9Arr);
        for (int i = 0; i < list.size(); i++) {
            C0607Ga.a aVar = C0607Ga.a.e;
            AbstractC3581z9[] abstractC3581z9Arr2 = {new C0607Ga(aVar), new C1304b9(i, Hl0.c), new W9(((com.android.tools.r8.graph.E0) list.get(i)).e), new C0607Ga(aVar), new G9(ByteCodes.invokespecial, b1A.a(((com.android.tools.r8.graph.E0) list.get(i)).e, b1A.a(b1A.E1, new I2[0]), b1A.c1), false), new B8(LN.b)};
            AbstractC3415xU.a(6, abstractC3581z9Arr2);
            c0599FuG.a(6, abstractC3581z9Arr2);
        }
        G9 g9 = new G9(ByteCodes.invokestatic, b1A.T4.a, false);
        G9 g92 = new G9(ByteCodes.invokeinterface, b1A.a(b1A.M2, b1A.a(b1A.D5, new I2[0]), b1A.c("iterator")), true);
        Hl0 hl0 = Hl0.b;
        AbstractC3581z9[] abstractC3581z9Arr3 = {g9, g92, k92, new C3328wa(hl0)};
        AbstractC3415xU.a(4, abstractC3581z9Arr3);
        c0599FuG.a(4, abstractC3581z9Arr3);
        K9 k93 = new K9();
        AbstractC3581z9[] abstractC3581z9Arr4 = {k93, C1982i9.T().a((EX) InterfaceC1376br.b(b1A.o3)).a(), new C1027Ua(hl0, 0), new W9(b1A.L2), new C0607Ga(C0607Ga.a.e), new P9(hl0, 0), new G9(ByteCodes.invokevirtual, b1A.w4.b, false), new P9(hl0, 0), new G9(ByteCodes.invokespecial, b1A.a(b1A.L2, b1A.a(b1A.E1, b1A.Y1, b1A.o3), b1A.c1), false), new C1134Ya()};
        AbstractC3415xU.a(10, abstractC3581z9Arr4);
        c0599FuG.a(10, abstractC3581z9Arr4);
        return new com.android.tools.r8.graph.G(null, 5, 1, c0599FuG.a(), new Ec0(new C1164Za(k9, k92, new Ec0(b1A.o3), new Ec0(k93))), S40.e);
    }
}
