package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.AbstractC0275i0;
import com.android.tools.r8.graph.B1;
import com.android.tools.r8.graph.C0409w2;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.E2;
import com.android.tools.r8.graph.E4;
import com.android.tools.r8.graph.I2;
import com.android.tools.r8.internal.AbstractC0695Iu;
import com.android.tools.r8.internal.AbstractC3415xU;
import com.android.tools.r8.internal.AbstractC3581z9;
import com.android.tools.r8.internal.C0607Ga;
import com.android.tools.r8.internal.C0611Ge;
import com.android.tools.r8.internal.C0987Sx;
import com.android.tools.r8.internal.C1134Ya;
import com.android.tools.r8.internal.C1446cb;
import com.android.tools.r8.internal.C1982i9;
import com.android.tools.r8.internal.C2826r9;
import com.android.tools.r8.internal.C3421xa;
import com.android.tools.r8.internal.EnumC2621ou;
import com.android.tools.r8.internal.G9;
import com.android.tools.r8.internal.Hl0;
import com.android.tools.r8.internal.InterfaceC1376br;
import com.android.tools.r8.internal.K9;
import com.android.tools.r8.internal.LO;
import com.android.tools.r8.internal.P9;
import com.android.tools.r8.internal.S40;
import com.android.tools.r8.internal.W9;
import com.android.tools.r8.synthesis.S;
import com.sun.tools.javac.jvm.ByteCodes;
import java.util.function.Consumer;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public abstract class I0 {
    public static AbstractC0275i0 a(B1 b1, C0409w2 c0409w2) {
        K9 k9 = new K9();
        K9 k92 = new K9();
        K9 k93 = new K9();
        K9 k94 = new K9();
        I2 i2 = c0409w2.f;
        Hl0 hl0 = Hl0.b;
        AbstractC0695Iu abstractC0695IuA = AbstractC0695Iu.a(k9, new P9(hl0, 0), new C2826r9(EnumC2621ou.b, hl0, k93), k92, new W9(b1.e("Ljava/lang/ClassCastException;")), new C0607Ga(C0607Ga.a.e), new G9(ByteCodes.invokespecial, b1.a(b1.e("Ljava/lang/ClassCastException;"), b1.a(b1.E1, new I2[0]), b1.c("<init>")), false), new C1134Ya(), k93, new C1982i9(new C0987Sx(new int[]{0}, new InterfaceC1376br[]{InterfaceC1376br.b(b1.a2)})), new C3421xa(), k94, new AbstractC3581z9[0]);
        S40 s40 = S40.e;
        return new com.android.tools.r8.graph.G(i2, 2, 1, abstractC0695IuA, s40, s40);
    }

    public static H0 b(final C0421y c0421y, LO lo, C0611Ge c0611Ge) {
        final B1 b1A = c0421y.a();
        final E2 e2A = b1A.a(b1A.E1, b1A.a2);
        A5 a5B = c0421y.a.g().b(new com.android.tools.r8.synthesis.I() { // from class: com.android.tools.r8.ir.optimize.I0$$ExternalSyntheticLambda5
            @Override // com.android.tools.r8.synthesis.I
            public final S.b a(com.android.tools.r8.synthesis.S s) {
                return s.C;
            }
        }, c0611Ge.a(), c0421y, new Consumer() { // from class: com.android.tools.r8.ir.optimize.I0$$ExternalSyntheticLambda6
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                I0.f(c0421y, b1A, e2A, (com.android.tools.r8.synthesis.N) obj);
            }
        });
        lo.s(a5B, c0611Ge.c);
        return new H0(a5B);
    }

    public static void c(C0421y c0421y, final B1 b1, E2 e2, com.android.tools.r8.synthesis.N n) {
        n.h = E4.b(4105, false);
        n.f = C1446cb.i;
        com.android.tools.r8.androidapi.f fVar = c0421y.U;
        n.l = fVar;
        n.m = fVar;
        n.g = new com.android.tools.r8.synthesis.M() { // from class: com.android.tools.r8.ir.optimize.I0$$ExternalSyntheticLambda0
            @Override // com.android.tools.r8.synthesis.M
            public final AbstractC0275i0 a(C0409w2 c0409w2) {
                return I0.c(b1, c0409w2);
            }
        };
        n.e = e2;
    }

    public static void d(C0421y c0421y, final B1 b1, E2 e2, com.android.tools.r8.synthesis.N n) {
        n.h = E4.b(4105, false);
        n.f = C1446cb.i;
        com.android.tools.r8.androidapi.f fVar = c0421y.U;
        n.l = fVar;
        n.m = fVar;
        n.g = new com.android.tools.r8.synthesis.M() { // from class: com.android.tools.r8.ir.optimize.I0$$ExternalSyntheticLambda9
            @Override // com.android.tools.r8.synthesis.M
            public final AbstractC0275i0 a(C0409w2 c0409w2) {
                return I0.d(b1, c0409w2);
            }
        };
        n.e = e2;
    }

    public static void e(C0421y c0421y, final B1 b1, E2 e2, com.android.tools.r8.synthesis.N n) {
        n.h = E4.b(4105, false);
        n.f = C1446cb.i;
        com.android.tools.r8.androidapi.f fVar = c0421y.U;
        n.l = fVar;
        n.m = fVar;
        n.g = new com.android.tools.r8.synthesis.M() { // from class: com.android.tools.r8.ir.optimize.I0$$ExternalSyntheticLambda2
            @Override // com.android.tools.r8.synthesis.M
            public final AbstractC0275i0 a(C0409w2 c0409w2) {
                return I0.e(b1, c0409w2);
            }
        };
        n.e = e2;
    }

    public static void f(C0421y c0421y, final B1 b1, E2 e2, com.android.tools.r8.synthesis.N n) {
        n.h = E4.b(4105, false);
        n.f = C1446cb.i;
        com.android.tools.r8.androidapi.f fVar = c0421y.U;
        n.l = fVar;
        n.m = fVar;
        n.g = new com.android.tools.r8.synthesis.M() { // from class: com.android.tools.r8.ir.optimize.I0$$ExternalSyntheticLambda1
            @Override // com.android.tools.r8.synthesis.M
            public final AbstractC0275i0 a(C0409w2 c0409w2) {
                return I0.f(b1, c0409w2);
            }
        };
        n.e = e2;
    }

    public static AbstractC0275i0 c(B1 b1, C0409w2 c0409w2) {
        K9 k9 = new K9();
        I2 i2 = c0409w2.f;
        AbstractC0695Iu abstractC0695IuA = AbstractC0695Iu.a(k9, new W9(b1.e("Ljava/lang/IncompatibleClassChangeError;")), new C0607Ga(C0607Ga.a.e), new G9(ByteCodes.invokespecial, b1.a(b1.e("Ljava/lang/IncompatibleClassChangeError;"), b1.a(b1.E1, new I2[0]), b1.c("<init>")), false), new C1134Ya());
        S40 s40 = S40.e;
        return new com.android.tools.r8.graph.G(i2, 2, 0, abstractC0695IuA, s40, s40);
    }

    public static AbstractC0275i0 d(B1 b1, C0409w2 c0409w2) {
        K9 k9 = new K9();
        I2 i2 = c0409w2.f;
        AbstractC0695Iu abstractC0695IuA = AbstractC0695Iu.a(k9, new W9(b1.e("Ljava/lang/NoSuchMethodError;")), new C0607Ga(C0607Ga.a.e), new G9(ByteCodes.invokespecial, b1.a(b1.e("Ljava/lang/NoSuchMethodError;"), b1.a(b1.E1, new I2[0]), b1.c("<init>")), false), new C1134Ya());
        S40 s40 = S40.e;
        return new com.android.tools.r8.graph.G(i2, 2, 0, abstractC0695IuA, s40, s40);
    }

    public static AbstractC0275i0 e(B1 b1, C0409w2 c0409w2) {
        K9 k9 = new K9();
        K9 k92 = new K9();
        I2 i2 = c0409w2.f;
        AbstractC0695Iu abstractC0695IuA = AbstractC0695Iu.a(k9, new W9(b1.e("Ljava/lang/RuntimeException;")), new C0607Ga(C0607Ga.a.e), new P9(Hl0.b, 0), new G9(ByteCodes.invokespecial, b1.a(b1.e("Ljava/lang/RuntimeException;"), b1.a(b1.E1, b1.Y1), b1.c("<init>")), false), new C1134Ya(), k92);
        S40 s40 = S40.e;
        return new com.android.tools.r8.graph.G(i2, 3, 1, abstractC0695IuA, s40, s40);
    }

    public static AbstractC0275i0 f(B1 b1, C0409w2 c0409w2) {
        K9 k9 = new K9();
        K9 k92 = new K9();
        K9 k93 = new K9();
        K9 k94 = new K9();
        I2 i2 = c0409w2.f;
        Hl0 hl0 = Hl0.b;
        Object[] objArrA = AbstractC3415xU.a(11, new Object[]{k9, new P9(hl0, 0), new C2826r9(EnumC2621ou.b, hl0, k93), k92, new P9(hl0, 0), new G9(ByteCodes.invokevirtual, b1.a(b1.a2, b1.a(b1.Y1, new I2[0]), b1.c("toString")), false), new C0607Ga(C0607Ga.a.c), k93, new C1982i9(new C0987Sx(new int[]{0}, new InterfaceC1376br[]{InterfaceC1376br.b(b1.a2)})), new C3421xa(), k94});
        AbstractC0695Iu abstractC0695IuB = AbstractC0695Iu.b(objArrA.length, objArrA);
        S40 s40 = S40.e;
        return new com.android.tools.r8.graph.G(i2, 1, 1, abstractC0695IuB, s40, s40);
    }

    public static AbstractC0275i0 b(B1 b1, C0409w2 c0409w2) {
        K9 k9 = new K9();
        I2 i2 = c0409w2.f;
        AbstractC0695Iu abstractC0695IuA = AbstractC0695Iu.a(k9, new W9(b1.e("Ljava/lang/IllegalAccessError;")), new C0607Ga(C0607Ga.a.e), new G9(ByteCodes.invokespecial, b1.a(b1.e("Ljava/lang/IllegalAccessError;"), b1.a(b1.E1, new I2[0]), b1.c("<init>")), false), new C1134Ya());
        S40 s40 = S40.e;
        return new com.android.tools.r8.graph.G(i2, 2, 0, abstractC0695IuA, s40, s40);
    }

    public static H0 c(final C0421y c0421y, J0 j0, C0611Ge c0611Ge) {
        final B1 b1A = c0421y.a();
        final E2 e2A = b1A.a(b1A.u3, new I2[0]);
        A5 a5B = c0421y.a.g().b(new com.android.tools.r8.synthesis.I() { // from class: com.android.tools.r8.ir.optimize.I0$$ExternalSyntheticLambda10
            @Override // com.android.tools.r8.synthesis.I
            public final S.b a(com.android.tools.r8.synthesis.S s) {
                return s.G;
            }
        }, c0611Ge.a(), c0421y, new Consumer() { // from class: com.android.tools.r8.ir.optimize.I0$$ExternalSyntheticLambda11
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                I0.d(c0421y, b1A, e2A, (com.android.tools.r8.synthesis.N) obj);
            }
        });
        j0.t(a5B, c0611Ge.c);
        return new H0(a5B);
    }

    public static H0 d(final C0421y c0421y, J0 j0, C0611Ge c0611Ge) {
        final B1 b1A = c0421y.a();
        final E2 e2A = b1A.a(b1A.m3, b1A.Y1);
        A5 a5B = c0421y.a.g().b(new com.android.tools.r8.synthesis.I() { // from class: com.android.tools.r8.ir.optimize.I0$$ExternalSyntheticLambda16
            @Override // com.android.tools.r8.synthesis.I
            public final S.b a(com.android.tools.r8.synthesis.S s) {
                return s.H;
            }
        }, c0611Ge.a(), c0421y, new Consumer() { // from class: com.android.tools.r8.ir.optimize.I0$$ExternalSyntheticLambda17
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                I0.e(c0421y, b1A, e2A, (com.android.tools.r8.synthesis.N) obj);
            }
        });
        j0.o(a5B, c0611Ge.c);
        return new H0(a5B);
    }

    public static void b(C0421y c0421y, final B1 b1, E2 e2, com.android.tools.r8.synthesis.N n) {
        n.h = E4.b(4105, false);
        n.f = C1446cb.i;
        com.android.tools.r8.androidapi.f fVar = c0421y.U;
        n.l = fVar;
        n.m = fVar;
        n.g = new com.android.tools.r8.synthesis.M() { // from class: com.android.tools.r8.ir.optimize.I0$$ExternalSyntheticLambda8
            @Override // com.android.tools.r8.synthesis.M
            public final AbstractC0275i0 a(C0409w2 c0409w2) {
                return I0.b(b1, c0409w2);
            }
        };
        n.e = e2;
    }

    public static H0 a(final C0421y c0421y, LO lo, C0611Ge c0611Ge) {
        final B1 b1A = c0421y.a();
        final E2 e2A = b1A.a(b1A.E1, b1A.a2);
        A5 a5B = c0421y.a.g().b(new com.android.tools.r8.synthesis.I() { // from class: com.android.tools.r8.ir.optimize.I0$$ExternalSyntheticLambda14
            @Override // com.android.tools.r8.synthesis.I
            public final S.b a(com.android.tools.r8.synthesis.S s) {
                return s.D;
            }
        }, c0611Ge.a(), c0421y, new Consumer() { // from class: com.android.tools.r8.ir.optimize.I0$$ExternalSyntheticLambda15
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                I0.a(c0421y, b1A, e2A, (com.android.tools.r8.synthesis.N) obj);
            }
        });
        lo.u(a5B, c0611Ge.c);
        return new H0(a5B);
    }

    public static H0 b(final C0421y c0421y, J0 j0, C0611Ge c0611Ge) {
        final B1 b1A = c0421y.a();
        final E2 e2A = b1A.a(b1A.s3, new I2[0]);
        A5 a5B = c0421y.a.g().b(new com.android.tools.r8.synthesis.I() { // from class: com.android.tools.r8.ir.optimize.I0$$ExternalSyntheticLambda3
            @Override // com.android.tools.r8.synthesis.I
            public final S.b a(com.android.tools.r8.synthesis.S s) {
                return s.F;
            }
        }, c0611Ge.a(), c0421y, new Consumer() { // from class: com.android.tools.r8.ir.optimize.I0$$ExternalSyntheticLambda4
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                I0.c(c0421y, b1A, e2A, (com.android.tools.r8.synthesis.N) obj);
            }
        });
        j0.v(a5B, c0611Ge.c);
        return new H0(a5B);
    }

    public static void a(C0421y c0421y, final B1 b1, E2 e2, com.android.tools.r8.synthesis.N n) {
        n.h = E4.b(4105, false);
        n.f = C1446cb.i;
        com.android.tools.r8.androidapi.f fVar = c0421y.U;
        n.l = fVar;
        n.m = fVar;
        n.g = new com.android.tools.r8.synthesis.M() { // from class: com.android.tools.r8.ir.optimize.I0$$ExternalSyntheticLambda7
            @Override // com.android.tools.r8.synthesis.M
            public final AbstractC0275i0 a(C0409w2 c0409w2) {
                return I0.a(b1, c0409w2);
            }
        };
        n.e = e2;
    }

    public static H0 a(final C0421y c0421y, J0 j0, C0611Ge c0611Ge) {
        final B1 b1A = c0421y.a();
        final E2 e2A = b1A.a(b1A.p3, new I2[0]);
        A5 a5B = c0421y.a.g().b(new com.android.tools.r8.synthesis.I() { // from class: com.android.tools.r8.ir.optimize.I0$$ExternalSyntheticLambda12
            @Override // com.android.tools.r8.synthesis.I
            public final S.b a(com.android.tools.r8.synthesis.S s) {
                return s.E;
            }
        }, c0611Ge.a(), c0421y, new Consumer() { // from class: com.android.tools.r8.ir.optimize.I0$$ExternalSyntheticLambda13
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                I0.b(c0421y, b1A, e2A, (com.android.tools.r8.synthesis.N) obj);
            }
        });
        j0.e(a5B, c0611Ge.c);
        return new H0(a5B);
    }
}
