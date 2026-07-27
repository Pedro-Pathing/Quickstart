package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.ClassFileConsumer;
import com.android.tools.r8.DataResource;
import com.android.tools.r8.FeatureSplit;
import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.AbstractC0232e;
import com.android.tools.r8.graph.B1;
import com.android.tools.r8.graph.C0285j;
import com.android.tools.r8.graph.C0286j0;
import com.android.tools.r8.graph.C0291j1;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.D2;
import com.android.tools.r8.graph.F2;
import com.android.tools.r8.graph.I2;
import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.AbstractC1308bC;
import com.android.tools.r8.internal.AbstractC3250vj0;
import com.android.tools.r8.internal.C0806Ml;
import com.android.tools.r8.internal.C0874Ot;
import com.android.tools.r8.internal.C0929Qv;
import com.android.tools.r8.internal.C0956Rv;
import com.android.tools.r8.internal.C0996Tg;
import com.android.tools.r8.internal.C1116Xj;
import com.android.tools.r8.internal.C1446cb;
import com.android.tools.r8.internal.C2384mC;
import com.android.tools.r8.internal.C2506nd;
import com.android.tools.r8.internal.C2929sC;
import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.internal.C3200vB;
import com.android.tools.r8.internal.H5;
import com.android.tools.r8.internal.InterfaceC1160Yw;
import com.android.tools.r8.internal.K5;
import com.android.tools.r8.internal.L5;
import com.android.tools.r8.shaking.C3877i;
import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;
import dk.sgjesse.r8api.DescriptorUtils;
import java.util.function.BiConsumer;
import java.util.function.Predicate;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public abstract class r0 {
    public static final /* synthetic */ boolean a = true;

    /* JADX WARN: Multi-variable type inference failed */
    public static void a(C0421y c0421y, C0874Ot c0874Ot) {
        boolean zA;
        D2 d2B;
        C3877i c3877i = (C3877i) c0421y.g();
        C3200vB c3200vBM = c0421y.M();
        c3877i.getClass();
        if (c3200vBM.j instanceof ClassFileConsumer) {
            if (c3877i.I == null) {
                synchronized (c3877i) {
                    if (c3877i.I == null) {
                        for (D2 d2 : c3877i.d()) {
                            if ((d2.v != null) != false) {
                                c3877i.I = (C1446cb) com.android.tools.r8.utils.structural.s.c(c3877i.I, d2.K1());
                            }
                        }
                        if (!C3877i.J && c3877i.I == null) {
                            throw new AssertionError();
                        }
                    }
                }
            }
            C1446cb c1446cb = c3877i.I;
            boolean z = C3200vB.Y1;
            if (!z && !(c3200vBM.j instanceof ClassFileConsumer)) {
                throw new AssertionError();
            }
            if (!z && !(c3200vBM.j instanceof ClassFileConsumer)) {
                throw new AssertionError();
            }
            zA = c1446cb.a(C1446cb.f);
        } else {
            zA = true;
        }
        if (zA) {
            C3658a c3658a = new C3658a();
            A5 a5I = c0874Ot.i();
            L5 l5T = c0874Ot.t();
            while (l5T.c.hasNext()) {
                H5 h5 = (H5) l5T.c.next();
                l5T.d = h5;
                K5 k5A = h5.a(c0874Ot);
                while (k5A.hasNext()) {
                    AbstractC1308bC abstractC1308bC = (AbstractC1308bC) k5A.a(new Predicate() { // from class: com.android.tools.r8.ir.optimize.r0$$ExternalSyntheticLambda1
                        @Override // java.util.function.Predicate
                        public final boolean test(Object obj) {
                            return r0.a((AbstractC1076Vw) obj);
                        }
                    });
                    if (abstractC1308bC != null) {
                        if (abstractC1308bC.a2()) {
                            C2384mC c2384mCG0 = abstractC1308bC.g0();
                            BiConsumer biConsumerA = a(c0421y, c0874Ot, l5T, k5A, abstractC1308bC, c3658a);
                            B1 b1A = c0421y.a();
                            if (c2384mCG0.U2() == b1A.y4.b) {
                                boolean z2 = a;
                                if (!z2 && c2384mCG0.c.size() != 1) {
                                    throw new AssertionError();
                                }
                                C3161ul0 c3161ul0I = c2384mCG0.b(0).i();
                                if (!c3161ul0I.y() && !c3161ul0I.l() && (!c2384mCG0.d1() || !c2384mCG0.d().y())) {
                                    AbstractC1076Vw abstractC1076Vw = c3161ul0I.c;
                                    abstractC1076Vw.getClass();
                                    I2 i2R0 = null;
                                    if (abstractC1076Vw instanceof C0806Ml) {
                                        F2 f2 = c3161ul0I.c.M().j;
                                        f2.getClass();
                                        if (f2 instanceof I2) {
                                            i2R0 = c3161ul0I.c.M().j.r0();
                                        }
                                    } else if (c3161ul0I.c.B1()) {
                                        String string = c3161ul0I.c.I().L2().toString();
                                        String strI = C1116Xj.F(string) ? C1116Xj.I(string) : null;
                                        if (strI == null && string.startsWith("[") && string.endsWith(RobotConfigFileManager.FILE_LIST_COMMAND_DELIMITER)) {
                                            strI = string.replace(DescriptorUtils.JAVA_PACKAGE_SEPARATOR, DataResource.SEPARATOR);
                                        }
                                        if (strI != null && strI.indexOf(46) <= 0) {
                                            i2R0 = b1A.e(strI);
                                            if (!i2R0.U0()) {
                                                continue;
                                            }
                                        }
                                    } else {
                                        continue;
                                    }
                                    if (i2R0 == null) {
                                        continue;
                                    } else {
                                        com.android.tools.r8.graph.E0 e0C = ((C3877i) c0421y.g()).c(i2R0.a(b1A));
                                        if (e0C != null && e0C.d(c0421y)) {
                                            C2506nd c2506nd = ((C3877i) c0421y.g()).g;
                                            if (AbstractC0232e.a(e0C, a5I, c0421y, (C0285j) c0421y.g()).b()) {
                                                continue;
                                            } else {
                                                if (!z2 && e0C.a0()) {
                                                    D2 d2X = e0C.X();
                                                    c2506nd.getClass();
                                                    com.android.tools.r8.synthesis.J jG = c0421y.a.g();
                                                    FeatureSplit featureSplitA = c2506nd.a(d2X.getType(), jG);
                                                    if (!featureSplitA.isBase() && featureSplitA != c2506nd.a(a5I.getReference(), jG)) {
                                                        throw new AssertionError();
                                                    }
                                                }
                                                biConsumerA.accept(i2R0, e0C);
                                            }
                                        }
                                    }
                                }
                            } else {
                                continue;
                            }
                        } else {
                            C2929sC c2929sCI0 = abstractC1308bC.i0();
                            BiConsumer biConsumerA2 = a(c0421y, c0874Ot, l5T, k5A, abstractC1308bC, c3658a);
                            B1 b1A2 = c0421y.a();
                            if (c2929sCI0.U2() == b1A2.p4.d) {
                                C3161ul0 c3161ul0V2 = c2929sCI0.V2();
                                if (!c3161ul0V2.y()) {
                                    AbstractC3250vj0 abstractC3250vj0T = c3161ul0V2.t();
                                    if (abstractC3250vj0T.w() || abstractC3250vj0T.r()) {
                                        if (!abstractC3250vj0T.F()) {
                                            I2 i2Q = abstractC3250vj0T.w() ? abstractC3250vj0T.b().Q() : abstractC3250vj0T.a().b(b1A2);
                                            I2 i2A = i2Q.a(b1A2);
                                            if (i2A.M0() && (d2B = D2.b(c0421y.d(i2A))) != null && (d2B.c(c0421y) || (!c3161ul0V2.l() && c3161ul0V2.c.D1()))) {
                                                N n = O.a(c0421y, i2A, a5I).a;
                                                n.getClass();
                                                if ((n == N.c) == false) {
                                                    biConsumerA2.accept(i2Q, d2B);
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            c3658a.a(c0421y, c0874Ot, C0996Tg.b());
            c0874Ot.y();
            if (!a && !c0874Ot.b((C0421y<?>) c0421y)) {
                throw new AssertionError();
            }
        }
    }

    public static /* synthetic */ boolean a(AbstractC1076Vw abstractC1076Vw) {
        return abstractC1076Vw.a2() || abstractC1076Vw.c2();
    }

    public static BiConsumer a(final C0421y c0421y, final C0874Ot c0874Ot, final L5 l5, final K5 k5, final AbstractC1308bC abstractC1308bC, final C3658a c3658a) {
        return new BiConsumer() { // from class: com.android.tools.r8.ir.optimize.r0$$ExternalSyntheticLambda0
            @Override // java.util.function.BiConsumer
            public final void accept(Object obj, Object obj2) {
                r0.a(abstractC1308bC, c0421y, c0874Ot, k5, c3658a, l5, (I2) obj, (com.android.tools.r8.graph.E0) obj2);
            }
        };
    }

    public static void a(AbstractC1308bC abstractC1308bC, C0421y c0421y, C0874Ot c0874Ot, InterfaceC1160Yw interfaceC1160Yw, C3658a c3658a, L5 l5, I2 i2, com.android.tools.r8.graph.E0 e0) {
        C0956Rv c0956Rv = null;
        if (abstractC1308bC.U2().d(c0421y.a().y4.b)) {
            if (e0.a0() && !((C3877i) c0421y.g()).f().a(c0874Ot.i(), e0.getType(), c0421y.a.g())) {
                return;
            }
            if (i2.M0() && e0.a(c0421y, c0874Ot.i())) {
                if (!e0.a0() || !c0421y.j()) {
                    return;
                }
                boolean z = C0956Rv.j;
                C0929Qv c0929Qv = new C0929Qv();
                c0929Qv.a = c0874Ot.a(AbstractC3250vj0.k(), (C0286j0) null);
                C0929Qv c0929Qv2 = (C0929Qv) c0929Qv.a();
                c0929Qv2.d = i2;
                c0929Qv2.b = abstractC1308bC.getPosition();
                c0956Rv = (C0956Rv) c0929Qv2.a(new C0956Rv(c0929Qv2.d, c0929Qv2.a));
            }
        }
        if (!abstractC1308bC.d1() || !abstractC1308bC.d().v()) {
            if (c0956Rv != null) {
                interfaceC1160Yw.e(c0956Rv);
                return;
            } else {
                interfaceC1160Yw.p();
                return;
            }
        }
        H5 h5B = abstractC1308bC.b();
        interfaceC1160Yw.a((C0421y<?>) c0421y, c0874Ot, i2, abstractC1308bC.m(), c3658a);
        if (c0956Rv != null) {
            if (h5B.x()) {
                interfaceC1160Yw.a(c0874Ot, l5, c0421y.M()).a(c0874Ot).add(c0956Rv);
            } else {
                interfaceC1160Yw.add(c0956Rv);
            }
        }
        if (c0421y.M().j instanceof ClassFileConsumer) {
            C0291j1 c0291j1V = c0874Ot.v();
            C3200vB c3200vBM = c0421y.M();
            if (!C3200vB.Y1) {
                if (!(c3200vBM.j instanceof ClassFileConsumer)) {
                    throw new AssertionError();
                }
            } else {
                c3200vBM.getClass();
            }
            c0291j1V.b(C1446cb.f);
        }
    }
}
