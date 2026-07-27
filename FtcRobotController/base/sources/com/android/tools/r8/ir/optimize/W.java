package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.C0291j1;
import com.android.tools.r8.graph.C0409w2;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.D2;
import com.android.tools.r8.graph.F5;
import com.android.tools.r8.graph.G4;
import com.android.tools.r8.graph.G5;
import com.android.tools.r8.graph.I2;
import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.AbstractC3424xb0;
import com.android.tools.r8.internal.AbstractC3644zm0;
import com.android.tools.r8.internal.Am0;
import com.android.tools.r8.internal.BX;
import com.android.tools.r8.internal.C0874Ot;
import com.android.tools.r8.internal.C0996Tg;
import com.android.tools.r8.internal.C1017Tr;
import com.android.tools.r8.internal.C1020Tt;
import com.android.tools.r8.internal.C1351bi;
import com.android.tools.r8.internal.C1581e10;
import com.android.tools.r8.internal.C1682f5;
import com.android.tools.r8.internal.C1711fS;
import com.android.tools.r8.internal.C2052iw;
import com.android.tools.r8.internal.C2298lM;
import com.android.tools.r8.internal.C2824r8;
import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.internal.C3200vB;
import com.android.tools.r8.internal.C3571z4;
import com.android.tools.r8.internal.CC;
import com.android.tools.r8.internal.EnumC3471y2;
import com.android.tools.r8.internal.Fh0;
import com.android.tools.r8.internal.G10;
import com.android.tools.r8.internal.H5;
import com.android.tools.r8.internal.H5$$ExternalSyntheticLambda0;
import com.android.tools.r8.internal.InterfaceC2144jw;
import com.android.tools.r8.internal.J5;
import com.android.tools.r8.internal.JO;
import com.android.tools.r8.internal.K5$$ExternalSyntheticLambda4;
import com.android.tools.r8.internal.L5;
import com.android.tools.r8.internal.RJ;
import com.android.tools.r8.internal.SY;
import com.android.tools.r8.internal.dk0$$ExternalSyntheticLambda1;
import com.android.tools.r8.shaking.C3877i;
import com.android.tools.r8.shaking.C3912o1;
import java.util.Iterator;
import java.util.ListIterator;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutorService;
import java.util.function.BiConsumer;
import java.util.function.BiPredicate;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.stream.Collectors;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public class W {
    public static final /* synthetic */ boolean i = true;
    public final C0421y a;
    public final C1020Tt b;
    public final RJ c;
    public final C2298lM d;
    public final C3661b0 e;
    public final ConcurrentHashMap f = new ConcurrentHashMap();
    public final Set g = AbstractC3424xb0.c();
    public final C1682f5 h;

    public W(C0421y c0421y, C1020Tt c1020Tt, RJ rj) {
        this.a = c0421y;
        this.b = c1020Tt;
        this.c = rj;
        ((C3877i) c0421y.g()).f();
        this.e = new C3661b0(c0421y);
        this.d = C2298lM.a(c0421y.A());
        C3200vB c3200vBM = c0421y.M();
        c3200vBM.getClass();
        this.h = c3200vBM.a(EnumC3471y2.w) ? new C1682f5(c0421y.M()) : null;
    }

    public static int b(C0874Ot c0874Ot) {
        Iterator<H5> it = c0874Ot.d.iterator();
        int i2 = 0;
        while (it.hasNext()) {
            for (AbstractC1076Vw abstractC1076Vw : it.next().k()) {
                if (!i && abstractC1076Vw.E1()) {
                    throw new AssertionError();
                }
                if (!abstractC1076Vw.l1() && !(abstractC1076Vw instanceof C3571z4) && (!abstractC1076Vw.L1() || abstractC1076Vw.S().L2().s().size() != 1)) {
                    if (!abstractC1076Vw.v2()) {
                        i2++;
                    }
                }
            }
        }
        return i2;
    }

    public AbstractC3644zm0 a(A5 a5, A5 a52) {
        C0421y c0421y = this.a;
        return ((C3877i) c0421y.g()).y.contains(a5.getReference()) ? new Am0(c0421y, a5, a52) : C1711fS.a;
    }

    public final void c(A5 a5, A5 a52) {
        C1020Tt c1020Tt = this.b;
        c1020Tt.a(a5);
        G5 g5 = c1020Tt.D;
        C0409w2 c0409w2U = a5.getReference();
        if (!F5.h && g5.c.containsKey(c0409w2U)) {
            throw new AssertionError();
        }
        g5.c.put(c0409w2U, a52);
        g5.g.add(c0409w2U);
        this.g.add(a5.getReference());
    }

    public final O a(C0874Ot c0874Ot) {
        C3200vB c3200vBM = this.a.M();
        if (!c3200vBM.T().b) {
            return O.c;
        }
        A5 a5I = c0874Ot.i();
        if (!a5I.e().n1()) {
            if (!a5I.a().f(this.a)) {
                C3912o1 c3912o1A = this.a.a(a5I);
                if (!c3912o1A.h(c3200vBM) && (!c3912o1A.c(c3200vBM) || !c3912o1A.j)) {
                    return O.c;
                }
                if (this.h == null) {
                    if (!i) {
                        C3200vB c3200vBM2 = this.a.M();
                        c3200vBM2.getClass();
                        if (c3200vBM2.a(EnumC3471y2.w)) {
                            throw new AssertionError();
                        }
                    }
                } else {
                    Iterator<H5> it = c0874Ot.d.iterator();
                    while (it.hasNext()) {
                        C2824r8 c2824r8I = it.next().i();
                        c2824r8I.getClass();
                        int i2 = 0;
                        while (i2 < c2824r8I.size()) {
                            I2 i22 = (I2) c2824r8I.b.get(i2);
                            c2824r8I.c.get(i2);
                            i2++;
                            com.android.tools.r8.graph.E0 e0D = this.a.d(i22);
                            if (e0D == null || e0D.b0()) {
                                if (!this.h.a.contains(i22)) {
                                    return O.c;
                                }
                            }
                        }
                    }
                }
                A5 a5I2 = c0874Ot.i();
                C3200vB c3200vBM3 = this.a.M();
                c3200vBM3.getClass();
                if (c3200vBM3.a(EnumC3471y2.w) && a(c0874Ot, a5I2)) {
                    return O.c;
                }
                O oA = O.d;
                X x = new X(this.a);
                for (AbstractC1076Vw abstractC1076Vw : c0874Ot.s()) {
                    O oA2 = abstractC1076Vw.a(x, a5I2);
                    N n = oA2.a;
                    n.getClass();
                    N n2 = N.c;
                    if (n == n2 && abstractC1076Vw.E1()) {
                        oA2 = O.d;
                    }
                    N n3 = oA2.a;
                    n3.getClass();
                    if (n3 == n2) {
                        return oA2;
                    }
                    oA = O.a(oA, oA2, (C0421y<?>) this.a);
                }
                return oA;
            }
        }
        return O.c;
    }

    public final /* synthetic */ boolean b(A5 a5, A5 a52) {
        if (!a5.e().N0() && !a5.e().E0().b()) {
            return false;
        }
        a5.c(this.a);
        this.b.c(a5);
        return true;
    }

    public static void b(final C0421y c0421y) {
        Iterator it = ((C3877i) c0421y.g()).e().iterator();
        while (it.hasNext()) {
            ((D2) it.next()).h(new Consumer() { // from class: com.android.tools.r8.ir.optimize.W$$ExternalSyntheticLambda3
                @Override // java.util.function.Consumer
                public final void accept(Object obj) {
                    W.a(c0421y, (A5) obj);
                }
            }, new Predicate() { // from class: com.android.tools.r8.ir.optimize.W$$ExternalSyntheticLambda2
                @Override // java.util.function.Predicate
                public final boolean test(Object obj) {
                    return W.a((C0291j1) obj);
                }
            });
        }
    }

    public static boolean a(C0874Ot c0874Ot, A5 a5) {
        I2 i2F1 = a5.e().F1();
        Iterator<H5> it = c0874Ot.d.iterator();
        while (it.hasNext()) {
            J5 j5H = it.next().H();
            while (j5H.hasNext()) {
                AbstractC1076Vw abstractC1076Vw = (AbstractC1076Vw) j5H.a(new K5$$ExternalSyntheticLambda4());
                if (abstractC1076Vw != null && i2F1.J0() && !((C3161ul0) abstractC1076Vw.c.get(0)).b((Set) null)) {
                    return true;
                }
            }
        }
        return false;
    }

    public final void a(A5 a5, C0874Ot c0874Ot, com.android.tools.r8.ir.optimize.info.y yVar, JO jo, Fh0 fh0) {
        a(a5, c0874Ot, yVar, jo, fh0, a(jo));
    }

    public final void a(A5 a5, C0874Ot c0874Ot, com.android.tools.r8.ir.optimize.info.y yVar, JO jo, Fh0 fh0, InterfaceC2144jw interfaceC2144jw) {
        H hA = a(a5, jo, this.a.M().T().f - b(c0874Ot), interfaceC2144jw);
        C2052iw c2052iw = new C2052iw(this.a, a5, c0874Ot, this.c, jo);
        if (!i && !C2052iw.g && !c2052iw.f.isEmpty()) {
            throw new AssertionError();
        }
        a(hA, a5, c0874Ot, yVar, c2052iw, jo, fh0);
    }

    public final InterfaceC2144jw a(JO jo) {
        final C1351bi c1351bi = new C1351bi(this.a, jo.c());
        return (InterfaceC2144jw) this.a.b(c1351bi, new Function() { // from class: com.android.tools.r8.ir.optimize.W$$ExternalSyntheticLambda4
            @Override // java.util.function.Function
            public final Object apply(Object obj) {
                return this.f$0.a(c1351bi, (C1017Tr) obj);
            }
        });
    }

    public final /* synthetic */ InterfaceC2144jw a(C1351bi c1351bi, C1017Tr c1017Tr) {
        return new C1581e10(this.a, c1351bi);
    }

    public H a(A5 a5, JO jo, int i2, InterfaceC2144jw interfaceC2144jw) {
        return new H(this.a, interfaceC2144jw, a5, jo, i2);
    }

    /* JADX WARN: Removed duplicated region for block: B:152:0x0393  */
    /* JADX WARN: Removed duplicated region for block: B:195:0x0116 A[SYNTHETIC] */
    /* JADX WARN: Removed duplicated region for block: B:200:0x00d6 A[SYNTHETIC] */
    /* JADX WARN: Removed duplicated region for block: B:201:0x0189 A[SYNTHETIC] */
    /* JADX WARN: Removed duplicated region for block: B:203:0x0174 A[SYNTHETIC] */
    /* JADX WARN: Removed duplicated region for block: B:58:0x014d  */
    /* JADX WARN: Removed duplicated region for block: B:59:0x0151  */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public final void a(com.android.tools.r8.ir.optimize.Y r26, com.android.tools.r8.graph.A5 r27, com.android.tools.r8.internal.C0874Ot r28, com.android.tools.r8.ir.optimize.info.y r29, com.android.tools.r8.internal.C2052iw r30, com.android.tools.r8.internal.JO r31, com.android.tools.r8.internal.Fh0 r32) {
        /*
            Method dump skipped, instruction units count: 1151
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.android.tools.r8.ir.optimize.W.a(com.android.tools.r8.ir.optimize.Y, com.android.tools.r8.graph.A5, com.android.tools.r8.internal.Ot, com.android.tools.r8.ir.optimize.info.y, com.android.tools.r8.internal.iw, com.android.tools.r8.internal.JO, com.android.tools.r8.internal.Fh0):void");
    }

    public static /* synthetic */ boolean a(H5 h5, H5 h52) {
        return h52 == h5;
    }

    public final void a(C0874Ot c0874Ot, L5 l5, H5 h5, C3658a c3658a, final Set set, Fh0 fh0) {
        H5 h52 = (H5) CC.a((ListIterator) l5);
        final Set setC = AbstractC3424xb0.c();
        a(l5, h5, new Consumer() { // from class: com.android.tools.r8.ir.optimize.W$$ExternalSyntheticLambda0
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                W.a(set, setC, (H5) obj);
            }
        });
        a(c0874Ot, l5, c3658a, setC);
        a(l5, h5, C0996Tg.b());
        a(c0874Ot, l5, setC, fh0);
        a(l5, h52, C0996Tg.b());
    }

    public static /* synthetic */ void a(Set set, Set set2, H5 h5) {
        if (set.contains(h5)) {
            return;
        }
        set2.add(h5);
    }

    public final void a(C0874Ot c0874Ot, L5 l5, Set set, Fh0 fh0) {
        new C3692q(this.a, 0).a(c0874Ot, l5, new H5$$ExternalSyntheticLambda0(set), fh0);
        if (!i && l5.c.hasNext()) {
            throw new AssertionError();
        }
    }

    public final void a(C0874Ot c0874Ot, L5 l5, C3658a c3658a, Set set) {
        G10 g10 = new G10(this.a);
        Objects.requireNonNull(set);
        g10.a(c0874Ot, l5, c3658a, new H5$$ExternalSyntheticLambda0(set));
        if (!i && l5.c.hasNext()) {
            throw new AssertionError();
        }
    }

    public static void a(ListIterator listIterator, H5 h5, Consumer consumer) {
        H5 h52;
        while (listIterator.hasPrevious() && (h52 = (H5) listIterator.previous()) != h5) {
            consumer.accept(h52);
        }
        if (!i && CC.a(listIterator) != h5) {
            throw new AssertionError();
        }
    }

    public final void a() {
        this.f.forEach(new BiConsumer() { // from class: com.android.tools.r8.ir.optimize.W$$ExternalSyntheticLambda10
            @Override // java.util.function.BiConsumer
            public final void accept(Object obj, Object obj2) {
                this.f$0.a((D2) obj, (SY) obj2);
            }
        });
        this.f.clear();
    }

    public final void a(D2 d2, SY sy) {
        sy.a(new BiPredicate() { // from class: com.android.tools.r8.ir.optimize.W$$ExternalSyntheticLambda8
            @Override // java.util.function.BiPredicate
            public final boolean test(Object obj, Object obj2) {
                return this.f$0.b((A5) obj, (A5) obj2);
            }
        });
        if (sy.b.isEmpty()) {
            return;
        }
        Set set = (Set) sy.j().map(new dk0$$ExternalSyntheticLambda1()).collect(Collectors.toSet());
        G4 g4V = d2.V();
        g4V.getClass();
        if (!set.isEmpty()) {
            g4V.b.a(set);
            g4V.c = C0291j1.v;
        }
        sy.forEach(new BiConsumer() { // from class: com.android.tools.r8.ir.optimize.W$$ExternalSyntheticLambda9
            @Override // java.util.function.BiConsumer
            public final void accept(Object obj, Object obj2) {
                this.f$0.c((A5) obj, (A5) obj2);
            }
        });
    }

    public static boolean a(C0291j1 c0291j1) {
        c0291j1.P0();
        return c0291j1.m.w();
    }

    public final void a(BX bx, Fh0 fh0, ExecutorService executorService) {
        BX bxA = bx.a(this.a);
        C2298lM c2298lM = this.d;
        C0421y c0421y = this.a;
        c2298lM.getClass();
        bxA.a.a(c2298lM.c(c0421y.A()).a(this.a, new Predicate() { // from class: com.android.tools.r8.ir.optimize.W$$ExternalSyntheticLambda1
            @Override // java.util.function.Predicate
            public final boolean test(Object obj) {
                return ((A5) obj).D().w();
            }
        }));
        this.d.d.clear();
        this.e.a(bx, fh0, executorService);
    }

    public static /* synthetic */ void a(C0421y c0421y, A5 a5) {
        if (!i && a5.e().j1() && a5.b(c0421y)) {
            throw new AssertionError();
        }
    }

    public static void a(C0421y c0421y) {
        Iterator it = c0421y.g().e().iterator();
        while (it.hasNext()) {
            for (C0291j1 c0291j1 : ((D2) it.next()).C1()) {
                if (c0291j1.j1()) {
                    c0291j1.P0();
                    c0291j1.m.A();
                }
            }
        }
    }
}
