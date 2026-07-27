package com.android.tools.r8.ir.optimize.info;

import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.C0257g1;
import com.android.tools.r8.graph.C0291j1;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.C0427y5;
import com.android.tools.r8.graph.F0;
import com.android.tools.r8.graph.H0;
import com.android.tools.r8.graph.InterfaceC0434z5;
import com.android.tools.r8.internal.AbstractC0564Em;
import com.android.tools.r8.internal.AbstractC0575Ew;
import com.android.tools.r8.internal.AbstractC0722Jn;
import com.android.tools.r8.internal.B1;
import com.android.tools.r8.internal.B7;
import com.android.tools.r8.internal.C2037il0;
import com.android.tools.r8.internal.InterfaceC2591oc;
import com.android.tools.r8.internal.Jb0;
import com.android.tools.r8.ir.optimize.O;
import java.util.BitSet;
import java.util.Set;
import java.util.function.Consumer;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public class C extends y {
    public static final /* synthetic */ boolean c = true;
    public static final C b = new C();

    public static void j(A5 a5) {
        Consumer consumer = new Consumer() { // from class: com.android.tools.r8.ir.optimize.info.C$$ExternalSyntheticLambda7
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                ((w) obj).R();
            }
        };
        if (a5.D().d()) {
            consumer.accept(a5.D().a());
        }
    }

    @Override // com.android.tools.r8.internal.InterfaceC3551yp
    public final void a(C0257g1 c0257g1, int i) {
    }

    @Override // com.android.tools.r8.internal.BO
    public final void b(C0291j1 c0291j1) {
        c0291j1.Z0().u &= -9;
    }

    @Override // com.android.tools.r8.internal.BO
    public final void c(C0291j1 c0291j1) {
        c0291j1.Z0().u |= 4;
    }

    @Override // com.android.tools.r8.internal.BO
    public final void d(C0291j1 c0291j1) {
    }

    @Override // com.android.tools.r8.internal.BO
    public final void e(C0291j1 c0291j1) {
        c0291j1.Z0().u |= 2;
    }

    public final void f(C0291j1 c0291j1) {
        c0291j1.Z0().u |= 1;
    }

    public final void g(C0291j1 c0291j1) {
        Consumer consumer = new Consumer() { // from class: com.android.tools.r8.ir.optimize.info.C$$ExternalSyntheticLambda12
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                ((w) obj).J();
            }
        };
        c0291j1.P0();
        if (c0291j1.m.d()) {
            c0291j1.P0();
            consumer.accept(c0291j1.m.a());
        }
    }

    public final void h(A5 a5) {
        b(new Consumer() { // from class: com.android.tools.r8.ir.optimize.info.C$$ExternalSyntheticLambda3
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                ((w) obj).P();
            }
        }, a5);
    }

    public final void i(A5 a5) {
        b(new Consumer() { // from class: com.android.tools.r8.ir.optimize.info.C$$ExternalSyntheticLambda18
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                ((w) obj).Q();
            }
        }, a5);
    }

    public final void k(A5 a5) {
        b(new Consumer() { // from class: com.android.tools.r8.ir.optimize.info.C$$ExternalSyntheticLambda1
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                ((w) obj).S();
            }
        }, a5);
    }

    public final void l(A5 a5) {
        b(new Consumer() { // from class: com.android.tools.r8.ir.optimize.info.C$$ExternalSyntheticLambda15
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                ((w) obj).T();
            }
        }, a5);
    }

    public final void m(A5 a5) {
        b(new Consumer() { // from class: com.android.tools.r8.ir.optimize.info.C$$ExternalSyntheticLambda0
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                ((w) obj).U();
            }
        }, a5);
    }

    public final void n(A5 a5) {
        b(new Consumer() { // from class: com.android.tools.r8.ir.optimize.info.C$$ExternalSyntheticLambda5
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                ((w) obj).V();
            }
        }, a5);
    }

    public final void o(A5 a5) {
        b(new Consumer() { // from class: com.android.tools.r8.ir.optimize.info.C$$ExternalSyntheticLambda16
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                ((w) obj).W();
            }
        }, a5);
    }

    public final void p(A5 a5) {
        b(new Consumer() { // from class: com.android.tools.r8.ir.optimize.info.C$$ExternalSyntheticLambda4
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                ((w) obj).X();
            }
        }, a5);
    }

    public final void q(A5 a5) {
        b(new Consumer() { // from class: com.android.tools.r8.ir.optimize.info.C$$ExternalSyntheticLambda6
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                ((w) obj).Y();
            }
        }, a5);
    }

    public final void r(A5 a5) {
        b(new Consumer() { // from class: com.android.tools.r8.ir.optimize.info.C$$ExternalSyntheticLambda19
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                ((w) obj).Z();
            }
        }, a5);
    }

    public final void s(A5 a5) {
        b(new Consumer() { // from class: com.android.tools.r8.ir.optimize.info.C$$ExternalSyntheticLambda11
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                ((w) obj).a0();
            }
        }, a5);
    }

    public final void t(A5 a5) {
        b(new Consumer() { // from class: com.android.tools.r8.ir.optimize.info.C$$ExternalSyntheticLambda8
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                ((w) obj).b0();
            }
        }, a5);
    }

    public static void a(C0257g1 c0257g1) {
        v vVar;
        synchronized (c0257g1) {
            vVar = (v) c0257g1.l.b();
            c0257g1.l = vVar;
        }
        vVar.b |= 1;
    }

    public final void d(A5 a5) {
        b(new Consumer() { // from class: com.android.tools.r8.ir.optimize.info.C$$ExternalSyntheticLambda13
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                ((w) obj).I();
            }
        }, a5);
    }

    @Override // com.android.tools.r8.internal.BO
    public final void b(C0291j1 c0291j1, BitSet bitSet) {
        c0291j1.Z0().o = bitSet;
    }

    @Override // com.android.tools.r8.internal.BO
    public final void c(A5 a5) {
        a5.e().Z0().u |= 32;
    }

    public final void e(A5 a5) {
        b(new Consumer() { // from class: com.android.tools.r8.ir.optimize.info.C$$ExternalSyntheticLambda14
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                ((w) obj).K();
            }
        }, a5);
    }

    public final void f(A5 a5) {
        b(new Consumer() { // from class: com.android.tools.r8.ir.optimize.info.C$$ExternalSyntheticLambda10
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                ((w) obj).L();
            }
        }, a5);
    }

    @Override // com.android.tools.r8.internal.BO
    public final void b(A5 a5, Jb0 jb0) {
        a5.e().Z0().p = jb0;
    }

    @Override // com.android.tools.r8.internal.InterfaceC3551yp
    public final void a(C0257g1 c0257g1, AbstractC0564Em abstractC0564Em) {
        v vVar;
        synchronized (c0257g1) {
            vVar = (v) c0257g1.l.b();
            c0257g1.l = vVar;
        }
        vVar.d = abstractC0564Em;
    }

    @Override // com.android.tools.r8.internal.BO
    public final void b(A5 a5, BitSet bitSet) {
        w wVarZ0 = a5.e().Z0();
        wVarZ0.getClass();
        if (!bitSet.isEmpty()) {
            wVarZ0.s = bitSet;
        } else {
            wVarZ0.s = null;
        }
    }

    public final void g(A5 a5) {
        b(new Consumer() { // from class: com.android.tools.r8.ir.optimize.info.C$$ExternalSyntheticLambda9
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                ((w) obj).M();
            }
        }, a5);
    }

    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.android.tools.r8.internal.InterfaceC3551yp
    public final void a(C0257g1 c0257g1, C0421y c0421y, B1 b1) {
        v vVar;
        if (!c) {
            InterfaceC0434z5 interfaceC0434z5B = c0257g1.b(c0421y);
            if (!y.a) {
                c0421y.getClass();
                ((F0) interfaceC0434z5B).f0();
                if (!c0421y.a(((C0427y5) interfaceC0434z5B).O()).a(c0421y, interfaceC0434z5B)) {
                    throw new AssertionError();
                }
            }
        }
        synchronized (c0257g1) {
            vVar = (v) c0257g1.l.b();
            c0257g1.l = vVar;
        }
        vVar.a(b1, c0257g1);
    }

    @Override // com.android.tools.r8.internal.BO
    public void b(A5 a5) {
        b(new Consumer() { // from class: com.android.tools.r8.ir.optimize.info.C$$ExternalSyntheticLambda17
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                ((w) obj).O();
            }
        }, a5);
    }

    public static void b(Consumer consumer, A5 a5) {
        if (a5.D().d()) {
            consumer.accept(a5.D().a());
        }
    }

    @Override // com.android.tools.r8.internal.BO
    public final void a(Set set, C0291j1 c0291j1) {
    }

    @Override // com.android.tools.r8.internal.BO
    public final void a(C0291j1 c0291j1, int i) {
        c0291j1.Z0().b(i);
    }

    @Override // com.android.tools.r8.internal.BO
    public final void a(C0291j1 c0291j1, C0421y c0421y, B1 b1) {
        c0291j1.Z0().a(b1, c0291j1);
    }

    @Override // com.android.tools.r8.internal.BO
    public final void a(C0291j1 c0291j1, C2037il0 c2037il0) {
        if (!c) {
            throw new AssertionError();
        }
    }

    @Override // com.android.tools.r8.internal.BO
    public final void a(C0421y c0421y, C0291j1 c0291j1, AbstractC0564Em abstractC0564Em) {
        c0291j1.Z0().a(c0421y, c0291j1, abstractC0564Em);
    }

    @Override // com.android.tools.r8.internal.BO
    public final void a(C0291j1 c0291j1, O o) {
        c0291j1.a(o);
    }

    @Override // com.android.tools.r8.internal.BO
    public final void a(A5 a5, B7 b7) {
        a5.e().Z0().l = b7;
    }

    @Override // com.android.tools.r8.internal.BO
    public final void a(A5 a5, InterfaceC2591oc interfaceC2591oc) {
    }

    public static void a(H0 h0) {
        h0.e().Z0().h = true;
    }

    @Override // com.android.tools.r8.internal.BO
    public final void a(A5 a5, AbstractC0722Jn abstractC0722Jn) {
        a5.e().Z0().a(abstractC0722Jn);
    }

    @Override // com.android.tools.r8.internal.BO
    public final void a(C0291j1 c0291j1, AbstractC0575Ew abstractC0575Ew) {
        c0291j1.Z0().m = abstractC0575Ew;
    }

    @Override // com.android.tools.r8.internal.BO
    public final void a(C0291j1 c0291j1) {
        c0291j1.Z0().u |= 128;
    }

    @Override // com.android.tools.r8.internal.BO
    public final void a(C0291j1 c0291j1, BitSet bitSet) {
        c0291j1.Z0().n = bitSet;
    }

    @Override // com.android.tools.r8.internal.BO
    public final void a(A5 a5, Jb0 jb0) {
        a5.e().Z0().q = jb0;
    }

    @Override // com.android.tools.r8.internal.BO
    public final void a(A5 a5, BitSet bitSet) {
        a5.e().Z0().a(bitSet);
    }

    public static void a(Consumer consumer, A5 a5) {
        if (a5.D().x()) {
            w wVarZ0 = a5.e().Z0();
            BitSet bitSet = (BitSet) wVarZ0.t.clone();
            consumer.accept(bitSet);
            if (bitSet == null || bitSet.isEmpty()) {
                bitSet = null;
            }
            wVarZ0.t = bitSet;
        }
    }

    @Override // com.android.tools.r8.internal.BO
    public final void a(A5 a5) {
        b(new Consumer() { // from class: com.android.tools.r8.ir.optimize.info.C$$ExternalSyntheticLambda2
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                ((w) obj).N();
            }
        }, a5);
    }
}
