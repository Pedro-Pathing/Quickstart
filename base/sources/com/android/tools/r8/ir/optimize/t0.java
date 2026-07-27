package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.errors.IllegalInvokeSuperToInterfaceOnDalvikDiagnostic;
import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.B1;
import com.android.tools.r8.graph.C0286j0;
import com.android.tools.r8.graph.C0409w2;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.H2;
import com.android.tools.r8.graph.I2;
import com.android.tools.r8.graph.K2;
import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.AbstractC2524nm;
import com.android.tools.r8.internal.AbstractC3250vj0;
import com.android.tools.r8.internal.C0759Kw;
import com.android.tools.r8.internal.C0874Ot;
import com.android.tools.r8.internal.C1494d40;
import com.android.tools.r8.internal.C1582e2;
import com.android.tools.r8.internal.C1780g2;
import com.android.tools.r8.internal.C2331lg0;
import com.android.tools.r8.internal.C2384mC;
import com.android.tools.r8.internal.C2474nC;
import com.android.tools.r8.internal.C2867rd;
import com.android.tools.r8.internal.C2937sJ;
import com.android.tools.r8.internal.C2947sS;
import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.internal.C3200vB;
import com.android.tools.r8.internal.C3502yP;
import com.android.tools.r8.internal.EnumC3471y2;
import com.android.tools.r8.internal.H5;
import com.android.tools.r8.internal.K5;
import com.android.tools.r8.internal.WS;
import com.android.tools.r8.internal.mu$$ExternalSyntheticLambda0;
import java.util.Collections;
import java.util.Iterator;
import java.util.function.Predicate;
import java.util.function.Supplier;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public abstract class t0 {
    public static final /* synthetic */ boolean a = true;

    public static boolean a(C0421y c0421y, C0874Ot c0874Ot) {
        c0421y.M().getClass();
        if (!c0874Ot.i.b(29)) {
            return false;
        }
        final I2 i2 = c0421y.a().a2;
        Iterator<H5> it = c0874Ot.d.iterator();
        boolean z = false;
        while (it.hasNext()) {
            K5 k5A = it.next().a(c0874Ot);
            while (k5A.hasNext()) {
                C0759Kw c0759Kw = (C0759Kw) k5A.a(new mu$$ExternalSyntheticLambda0());
                if (c0759Kw != null) {
                    AbstractC3250vj0 abstractC3250vj0T = ((C3161ul0) c0759Kw.c.get(0)).t();
                    abstractC3250vj0T.getClass();
                    if (abstractC3250vj0T instanceof C1494d40) {
                        k5A.a(c0874Ot, 0);
                    } else if (c0421y.o() && abstractC3250vj0T.a(new Predicate() { // from class: com.android.tools.r8.ir.optimize.t0$$ExternalSyntheticLambda6
                        @Override // java.util.function.Predicate
                        public final boolean test(Object obj) {
                            return t0.a(i2, (C2867rd) obj);
                        }
                    })) {
                        I2 i22 = c0759Kw.i;
                        C2947sS c2947sSN = abstractC3250vj0T.N();
                        i22.getClass();
                        AbstractC3250vj0 abstractC3250vj0A = AbstractC3250vj0.a(i22, c2947sSN, (C0421y<?>) c0421y);
                        if (abstractC3250vj0A.a(new Predicate() { // from class: com.android.tools.r8.ir.optimize.t0$$ExternalSyntheticLambda7
                            @Override // java.util.function.Predicate
                            public final boolean test(Object obj) {
                                return t0.b(i2, (C2867rd) obj);
                            }
                        }) && !abstractC3250vj0A.a(abstractC3250vj0T, (C0421y<?>) c0421y) && !abstractC3250vj0T.a(abstractC3250vj0A, (C0421y<?>) c0421y) && !abstractC3250vj0T.a(c0421y.U())) {
                            k5A.a(c0874Ot, 0);
                        }
                    }
                    z = true;
                }
            }
        }
        if (a || c0874Ot.b((C0421y<?>) c0421y)) {
            return z;
        }
        throw new AssertionError();
    }

    public static boolean b(I2 i2, C2867rd c2867rd) {
        return !c2867rd.Q().a(i2);
    }

    public static void b(C0874Ot c0874Ot, final C3200vB c3200vB) {
        c3200vB.getClass();
        if (c3200vB.a(EnumC3471y2.y) && c0874Ot.v().r1()) {
            K2 k2 = c0874Ot.v().getReference().i.f;
            if (k2.size() == 3) {
                I2[] i2Arr = k2.b;
                I2 i2 = i2Arr[0];
                I2 i22 = c3200vB.a.z1;
                if (i2 == i22 && i2Arr[1] == i22 && i2Arr[2].M0()) {
                    Iterator<H5> it = c0874Ot.d.iterator();
                    while (it.hasNext()) {
                        K5 k5A = it.next().a(c0874Ot);
                        AbstractC1076Vw abstractC1076Vw = (AbstractC1076Vw) k5A.a(new Predicate() { // from class: com.android.tools.r8.ir.optimize.t0$$ExternalSyntheticLambda4
                            @Override // java.util.function.Predicate
                            public final boolean test(Object obj) {
                                return t0.a(c3200vB, (AbstractC1076Vw) obj);
                            }
                        });
                        if (abstractC1076Vw != null) {
                            AbstractC1076Vw abstractC1076VwPrevious = k5A.previous();
                            if (!a && abstractC1076Vw != abstractC1076VwPrevious) {
                                throw new AssertionError();
                            }
                            C3161ul0 c3161ul0A = c0874Ot.a(AbstractC3250vj0.k(), (C0286j0) null);
                            C1582e2 c1582e2 = new C1582e2(c3161ul0A);
                            c1582e2.a(abstractC1076Vw.b());
                            c1582e2.b(abstractC1076Vw.getPosition());
                            k5A.add(c1582e2);
                            C1780g2 c1780g2 = new C1780g2(c3161ul0A);
                            c1780g2.a(abstractC1076Vw.b());
                            c1780g2.b(abstractC1076Vw.getPosition());
                            k5A.add(c1780g2);
                            return;
                        }
                    }
                }
            }
        }
    }

    public static boolean a(I2 i2, C2867rd c2867rd) {
        return !c2867rd.Q().a(i2);
    }

    public static void a(C0874Ot c0874Ot, C3200vB c3200vB) {
        AbstractC1076Vw abstractC1076Vw;
        c3200vB.getClass();
        if (c3200vB.a(EnumC3471y2.z)) {
            final B1 b1 = c3200vB.a;
            C2937sJ c2937sJ = new C2937sJ(new Supplier() { // from class: com.android.tools.r8.ir.optimize.t0$$ExternalSyntheticLambda1
                @Override // java.util.function.Supplier
                public final Object get() {
                    B1 b12 = b1;
                    return b12.a(b12.c("Ljava/lang/Long;"), b12.c("signum"), b12.B, new H2[]{b12.C});
                }
            });
            for (H5 h5 : c0874Ot.d) {
                K5 k5A = h5.a(c0874Ot);
                final AbstractC1076Vw abstractC1076Vw2 = (AbstractC1076Vw) k5A.a(new Predicate() { // from class: com.android.tools.r8.ir.optimize.t0$$ExternalSyntheticLambda2
                    @Override // java.util.function.Predicate
                    public final boolean test(Object obj) {
                        return t0.a((AbstractC1076Vw) obj);
                    }
                });
                if (abstractC1076Vw2 != null && (abstractC1076Vw2 instanceof C3502yP)) {
                    WS wsK2 = abstractC1076Vw2.A().K2();
                    WS ws = WS.f;
                    if (wsK2 == ws && abstractC1076Vw2.d() != null && (abstractC1076Vw = (AbstractC1076Vw) k5A.a(new Predicate() { // from class: com.android.tools.r8.ir.optimize.t0$$ExternalSyntheticLambda2
                        @Override // java.util.function.Predicate
                        public final boolean test(Object obj) {
                            return t0.a((AbstractC1076Vw) obj);
                        }
                    })) != null && (abstractC1076Vw.i1() || (abstractC1076Vw instanceof C2331lg0))) {
                        if (abstractC1076Vw.A().K2() == ws) {
                            Iterator<H5> it = h5.s().iterator();
                            while (true) {
                                if (it.hasNext()) {
                                    if (it.next().h().K2() == h5) {
                                        break;
                                    }
                                } else {
                                    C3161ul0 c3161ul0D = abstractC1076Vw2.d();
                                    for (C3161ul0 c3161ul0L2 : abstractC1076Vw.c) {
                                        while (c3161ul0L2 != c3161ul0D) {
                                            AbstractC1076Vw abstractC1076Vw3 = c3161ul0L2.c;
                                            if (abstractC1076Vw3 == null || !abstractC1076Vw3.i2()) {
                                                break;
                                            } else {
                                                c3161ul0L2 = abstractC1076Vw3.n0().L2();
                                            }
                                        }
                                        K5 k5A2 = h5.a(c0874Ot);
                                        k5A2.a(new Predicate() { // from class: com.android.tools.r8.ir.optimize.t0$$ExternalSyntheticLambda3
                                            @Override // java.util.function.Predicate
                                            public final boolean test(Object obj) {
                                                return t0.a(abstractC1076Vw2, (AbstractC1076Vw) obj);
                                            }
                                        });
                                        C2384mC c2384mC = new C2384mC((C0409w2) c2937sJ.a(c2937sJ.b), null, Collections.singletonList((C3161ul0) abstractC1076Vw2.c.get(0)));
                                        AbstractC1076Vw abstractC1076VwPrevious = k5A2.previous();
                                        boolean z = a;
                                        if (!z && abstractC1076Vw2 != abstractC1076VwPrevious) {
                                            throw new AssertionError();
                                        }
                                        H5 h5B = abstractC1076VwPrevious.b();
                                        if (h5B.x()) {
                                            H5 h5A = k5A2.a(c0874Ot);
                                            if (!z && !h5A.x()) {
                                                throw new AssertionError();
                                            }
                                            if (!z && h5B.x()) {
                                                throw new AssertionError();
                                            }
                                            k5A2 = h5B.a(c0874Ot, h5B.k().size() - 1);
                                        }
                                        c2384mC.b(abstractC1076Vw2.getPosition());
                                        k5A2.add(c2384mC);
                                        return;
                                    }
                                }
                            }
                        } else {
                            continue;
                        }
                    }
                }
            }
        }
    }

    public static /* synthetic */ boolean a(AbstractC1076Vw abstractC1076Vw, AbstractC1076Vw abstractC1076Vw2) {
        return abstractC1076Vw2 == abstractC1076Vw;
    }

    public static boolean a(C3200vB c3200vB, AbstractC1076Vw abstractC1076Vw) {
        return abstractC1076Vw.U1() && abstractC1076Vw.a0().U2().g == c3200vB.a.c1 && abstractC1076Vw.a0().c.size() == 4 && abstractC1076Vw.a0().c.stream().allMatch(new Predicate() { // from class: com.android.tools.r8.ir.optimize.t0$$ExternalSyntheticLambda5
            @Override // java.util.function.Predicate
            public final boolean test(Object obj) {
                return ((C3161ul0) obj).F();
            }
        });
    }

    public static boolean a(AbstractC1076Vw abstractC1076Vw) {
        return (abstractC1076Vw.E1() || abstractC1076Vw.i2()) ? false : true;
    }

    public static void a(C0874Ot c0874Ot, C3200vB c3200vB, C0421y c0421y) {
        boolean zIsInterface;
        AbstractC2524nm abstractC2524nm = c0421y.f;
        c3200vB.getClass();
        if (c3200vB.a(EnumC3471y2.w)) {
            A5 a5I = c0874Ot.i();
            abstractC2524nm.getClass();
            if (abstractC2524nm.a(a5I.s())) {
                return;
            }
            Iterator it = c0874Ot.b(new Predicate() { // from class: com.android.tools.r8.ir.optimize.t0$$ExternalSyntheticLambda0
                @Override // java.util.function.Predicate
                public final boolean test(Object obj) {
                    return ((AbstractC1076Vw) obj).b2();
                }
            }).iterator();
            while (it.hasNext()) {
                C0409w2 c0409w2U2 = ((C2474nC) it.next()).U2();
                I2 i2W0 = c0409w2U2.w0();
                i2W0.getClass();
                com.android.tools.r8.graph.E0 e0D = c0421y.d(i2W0);
                if (e0D == null) {
                    zIsInterface = false;
                } else {
                    zIsInterface = e0D.isInterface();
                }
                if (zIsInterface && !abstractC2524nm.a(c0409w2U2.w0())) {
                    c0421y.M().i.warning(new IllegalInvokeSuperToInterfaceOnDalvikDiagnostic(c0874Ot.i().A(), c0409w2U2.z0(), c0874Ot.i().b.d));
                }
            }
        }
    }
}
