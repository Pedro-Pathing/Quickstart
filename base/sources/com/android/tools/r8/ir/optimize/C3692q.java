package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.C0286j0;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.internal.AC;
import com.android.tools.r8.internal.AbstractC0564Em;
import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.AbstractC3250vj0;
import com.android.tools.r8.internal.AbstractC3424xb0;
import com.android.tools.r8.internal.C0621Gm;
import com.android.tools.r8.internal.C0874Ot;
import com.android.tools.r8.internal.C0996Tg;
import com.android.tools.r8.internal.C1105Wz;
import com.android.tools.r8.internal.C2026ig;
import com.android.tools.r8.internal.C2435mm;
import com.android.tools.r8.internal.C2445mu;
import com.android.tools.r8.internal.C2881rj0;
import com.android.tools.r8.internal.C2947sS;
import com.android.tools.r8.internal.C3026tJ;
import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.internal.C3571z4;
import com.android.tools.r8.internal.C5;
import com.android.tools.r8.internal.Fh0;
import com.android.tools.r8.internal.H5;
import com.android.tools.r8.internal.HX;
import com.android.tools.r8.internal.InterfaceC2382mA;
import com.android.tools.r8.internal.InterfaceC2472nA;
import com.android.tools.r8.internal.InterfaceC2517ni0;
import com.android.tools.r8.internal.InterfaceC2604oi0;
import com.android.tools.r8.internal.J5;
import com.android.tools.r8.internal.K5;
import com.android.tools.r8.internal.L5;
import com.android.tools.r8.internal.PX;
import com.android.tools.r8.internal.SW;
import com.android.tools.r8.internal.Y6;
import com.android.tools.r8.internal.fw$$ExternalSyntheticLambda5;
import com.android.tools.r8.internal.p3$$ExternalSyntheticLambda1;
import com.android.tools.r8.shaking.C3877i;
import java.util.ArrayList;
import java.util.IdentityHashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.Function;
import java.util.function.Predicate;

/* JADX INFO: renamed from: com.android.tools.r8.ir.optimize.q, reason: case insensitive filesystem */
/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public class C3692q {
    public static final /* synthetic */ boolean c = true;
    public final C0421y a;
    public final boolean b = false;

    public C3692q(C0421y<C3877i> c0421y) {
        this.a = c0421y;
    }

    public static /* synthetic */ boolean c(AbstractC1076Vw abstractC1076Vw, AbstractC1076Vw abstractC1076Vw2) {
        return abstractC1076Vw2 != abstractC1076Vw;
    }

    public void a(C0874Ot c0874Ot, Fh0 fh0) {
        a(c0874Ot, c0874Ot.t(), PX.b, fh0);
        c0874Ot.y();
        if (!c && !c0874Ot.b(this.a)) {
            throw new AssertionError();
        }
    }

    public final void b(final C0874Ot c0874Ot, C3686k c3686k) {
        Set setC = AbstractC3424xb0.c();
        IdentityHashMap identityHashMap = new IdentityHashMap();
        a(c0874Ot, c3686k, setC, identityHashMap, new Predicate() { // from class: com.android.tools.r8.ir.optimize.q$$ExternalSyntheticLambda3
            @Override // java.util.function.Predicate
            public final boolean test(Object obj) {
                return C3692q.a((C3677i) obj);
            }
        });
        a(c0874Ot, c3686k, setC, identityHashMap, new Predicate() { // from class: com.android.tools.r8.ir.optimize.q$$ExternalSyntheticLambda4
            @Override // java.util.function.Predicate
            public final boolean test(Object obj) {
                return C3692q.b((C3677i) obj);
            }
        });
        identityHashMap.forEach(new BiConsumer() { // from class: com.android.tools.r8.ir.optimize.q$$ExternalSyntheticLambda5
            @Override // java.util.function.BiConsumer
            public final void accept(Object obj, Object obj2) {
                C3692q.a(c0874Ot, (H5) obj, (Map) obj2);
            }
        });
        if (setC.isEmpty()) {
            return;
        }
        C2881rj0 c2881rj0 = new C2881rj0(this.a, c0874Ot, false);
        c2881rj0.b = this.b;
        c2881rj0.a(setC, C0996Tg.b());
    }

    public C3692q(C0421y c0421y, int i) {
        this.a = c0421y;
    }

    public final void a(C0874Ot c0874Ot, L5 l5, Predicate predicate, Fh0 fh0) {
        fh0.a("Insert assume instructions");
        fh0.a("Part 1: Compute assumed values");
        C3684j c3684j = new C3684j();
        while (l5.c.hasNext()) {
            H5 h5 = (H5) l5.c.next();
            l5.d = h5;
            if (predicate.test(h5)) {
                a(c0874Ot, l5, h5, c3684j);
            }
        }
        LinkedHashMap linkedHashMap = c3684j.a;
        C3686k c3686k = new C3686k(linkedHashMap);
        fh0.b();
        if (!linkedHashMap.isEmpty()) {
            fh0.a("Part 2: Remove redundant assume instructions");
            a(c3686k);
            fh0.b();
            fh0.a("Part 3: Compute dominated users");
            IdentityHashMap identityHashMapA = a(c0874Ot, c3686k);
            fh0.b();
            if (!linkedHashMap.isEmpty()) {
                fh0.a("Part 4: Remove redundant dominated assume instructions");
                c3686k.a(identityHashMapA);
                fh0.b();
                if (!linkedHashMap.isEmpty()) {
                    fh0.a("Part 5: Materialize assume instructions");
                    b(c0874Ot, c3686k);
                    fh0.b();
                }
            }
        }
        fh0.b();
    }

    /* JADX WARN: Removed duplicated region for block: B:106:0x0237  */
    /* JADX WARN: Removed duplicated region for block: B:12:0x0035  */
    /* JADX WARN: Removed duplicated region for block: B:167:0x02d7 A[SYNTHETIC] */
    /* JADX WARN: Removed duplicated region for block: B:170:0x028a A[SYNTHETIC] */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public final void a(com.android.tools.r8.internal.C0874Ot r21, com.android.tools.r8.internal.L5 r22, com.android.tools.r8.internal.H5 r23, com.android.tools.r8.ir.optimize.C3684j r24) {
        /*
            Method dump skipped, instruction units count: 902
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.android.tools.r8.ir.optimize.C3692q.a(com.android.tools.r8.internal.Ot, com.android.tools.r8.internal.L5, com.android.tools.r8.internal.H5, com.android.tools.r8.ir.optimize.j):void");
    }

    public static boolean b(C3677i c3677i) {
        AbstractC3675h abstractC3675h = c3677i.a;
        abstractC3675h.getClass();
        return abstractC3675h instanceof C3687l;
    }

    public static /* synthetic */ List b(AbstractC1076Vw abstractC1076Vw) {
        return new ArrayList();
    }

    public static boolean b(AbstractC1076Vw abstractC1076Vw, C3161ul0 c3161ul0) {
        AbstractC3250vj0 abstractC3250vj0T = c3161ul0.t();
        return abstractC3250vj0T.I() && abstractC3250vj0T.d().F() && !abstractC3250vj0T.N().e() && a(abstractC1076Vw, c3161ul0);
    }

    public static /* synthetic */ boolean b(AbstractC1076Vw abstractC1076Vw, AbstractC1076Vw abstractC1076Vw2) {
        return abstractC1076Vw2 != abstractC1076Vw;
    }

    public final boolean a(AbstractC1076Vw abstractC1076Vw, AbstractC0564Em abstractC0564Em, C3684j c3684j) {
        C3161ul0 c3161ul0D = abstractC1076Vw.d();
        if (abstractC0564Em.l()) {
            return false;
        }
        if (abstractC0564Em.j()) {
            c3684j.a(abstractC1076Vw, c3161ul0D);
            return true;
        }
        C0621Gm c0621GmA = abstractC0564Em.a();
        C0621Gm c0621GmA2 = AbstractC0564Em.a(this.a, c3161ul0D.t());
        if (!c0621GmA.b(this.a, c0621GmA2)) {
            return false;
        }
        if (!c0621GmA.b.N().f() && c0621GmA.a(C2947sS.h()).equals(c0621GmA2)) {
            if (!c && !c0621GmA.b.N().d()) {
                throw new AssertionError();
            }
            c3684j.a(abstractC1076Vw, c3161ul0D);
        } else {
            c3684j.a(abstractC1076Vw, c3161ul0D, c0621GmA);
        }
        return true;
    }

    public static void a(final C3686k c3686k) {
        c3686k.a(new InterfaceC2604oi0() { // from class: com.android.tools.r8.ir.optimize.q$$ExternalSyntheticLambda6
            @Override // com.android.tools.r8.internal.InterfaceC2604oi0
            public final boolean a(Object obj, Object obj2, Object obj3) {
                return C3692q.a(c3686k, (AbstractC1076Vw) obj, (C3161ul0) obj2, (C3677i) obj3);
            }
        });
    }

    public static boolean a(C3686k c3686k, AbstractC1076Vw abstractC1076Vw, C3161ul0 c3161ul0, C3677i c3677i) {
        AbstractC1076Vw abstractC1076VwM;
        if (c3677i.a()) {
            return false;
        }
        if (!c && !c3677i.b()) {
            throw new AssertionError();
        }
        if (c3161ul0.l() || (abstractC1076VwM = c3161ul0.m()) == abstractC1076Vw) {
            return false;
        }
        Map map = (Map) c3686k.a.get(abstractC1076VwM);
        C3677i c3677i2 = map != null ? (C3677i) map.get(c3161ul0) : null;
        if (c3677i2 == null) {
            return false;
        }
        if (c3677i2.b()) {
            return true;
        }
        c3677i.a(c3677i2.b.a(C2947sS.b()));
        return false;
    }

    public final IdentityHashMap a(C0874Ot c0874Ot, C3686k c3686k) {
        final IdentityHashMap identityHashMap = new IdentityHashMap();
        final C3026tJ c3026tJ = new C3026tJ(c0874Ot);
        final IdentityHashMap identityHashMap2 = new IdentityHashMap();
        c3686k.a(new InterfaceC2517ni0() { // from class: com.android.tools.r8.ir.optimize.q$$ExternalSyntheticLambda2
            @Override // com.android.tools.r8.internal.InterfaceC2517ni0
            public final Object a(Object obj, Object obj2, Object obj3) {
                return this.f$0.a(identityHashMap, c3026tJ, identityHashMap2, (AbstractC1076Vw) obj, (C3161ul0) obj2, (C3677i) obj3);
            }
        });
        return identityHashMap;
    }

    public final AbstractC3675h a(Map map, C3026tJ c3026tJ, Map map2, final AbstractC1076Vw abstractC1076Vw, final C3161ul0 c3161ul0, C3677i c3677i) {
        H5 h5B;
        AbstractC1076Vw next;
        C3677i c3677i2;
        Map map3 = (Map) map.get(abstractC1076Vw);
        if (map3 != null && (c3677i2 = (C3677i) map3.get(c3161ul0)) != null) {
            if (!c3677i.a() && c3677i2.b()) {
                return C3691p.a;
            }
            if (c3677i2.b()) {
                c3677i.c();
            }
            if (!c3677i.a() && c3677i2.a()) {
                c3677i.a(c3677i2.b.a(c3677i.b.d()));
            }
        }
        if (c3161ul0 == abstractC1076Vw.d()) {
            return C3687l.a;
        }
        H5 h5B2 = abstractC1076Vw.b();
        if (c3161ul0.b() == h5B2 && h5B2.h().L1() && !abstractC1076Vw.b().x()) {
            J5 j5H = abstractC1076Vw.b().H();
            if (!c3161ul0.l()) {
                j5H.a(new Predicate() { // from class: com.android.tools.r8.ir.optimize.q$$ExternalSyntheticLambda8
                    @Override // java.util.function.Predicate
                    public final boolean test(Object obj) {
                        return C3692q.a(c3161ul0, (AbstractC1076Vw) obj);
                    }
                });
                j5H.previous();
            }
            while (j5H.hasNext() && (next = j5H.next()) != abstractC1076Vw) {
                if (next.c.contains(c3161ul0) || next.T0().contains(c3161ul0)) {
                }
            }
            return C3688m.a;
        }
        if (abstractC1076Vw.M1()) {
            C2445mu c2445muT = abstractC1076Vw.T();
            if (c2445muT.O2()) {
                if (!C2445mu.k && !c2445muT.P2()) {
                    throw new AssertionError();
                }
                h5B = c2445muT.b(Y6.a(true));
            } else {
                h5B = c2445muT.S2();
            }
        } else {
            h5B = abstractC1076Vw.b();
            if (h5B.x()) {
                h5B = C5.a(h5B);
            }
        }
        if (!c && !c3161ul0.A() && !c3161ul0.b0().stream().anyMatch(new Predicate() { // from class: com.android.tools.r8.ir.optimize.q$$ExternalSyntheticLambda9
            @Override // java.util.function.Predicate
            public final boolean test(Object obj) {
                return C3692q.a(abstractC1076Vw, (AbstractC1076Vw) obj);
            }
        }) && !c3161ul0.F()) {
            throw new AssertionError();
        }
        final C2435mm c2435mmC = c3026tJ.a();
        Set set = (Set) map2.computeIfAbsent(h5B, new Function() { // from class: com.android.tools.r8.ir.optimize.q$$ExternalSyntheticLambda10
            @Override // java.util.function.Function
            public final Object apply(Object obj) {
                return C3692q.a(c2435mmC, (H5) obj);
            }
        });
        C3673g c3673g = new C3673g(c3161ul0);
        for (final AbstractC1076Vw abstractC1076Vw2 : c3161ul0.b0()) {
            if (abstractC1076Vw2 != abstractC1076Vw && set.contains(abstractC1076Vw2.b())) {
                if (abstractC1076Vw2.b() == h5B && h5B == h5B2) {
                    AbstractC1076Vw abstractC1076Vw3 = (AbstractC1076Vw) h5B2.H().a(new Predicate() { // from class: com.android.tools.r8.ir.optimize.q$$ExternalSyntheticLambda11
                        @Override // java.util.function.Predicate
                        public final boolean test(Object obj) {
                            return C3692q.a(abstractC1076Vw, abstractC1076Vw2, (AbstractC1076Vw) obj);
                        }
                    });
                    if (!c && abstractC1076Vw3 == null) {
                        throw new AssertionError();
                    }
                    if (abstractC1076Vw3 == abstractC1076Vw2) {
                        continue;
                    }
                }
                boolean z = C3673g.d;
                if (!z && !c3673g.a.b0().contains(abstractC1076Vw2)) {
                    throw new AssertionError();
                }
                if (!z && c3673g.b.contains(abstractC1076Vw2)) {
                    throw new AssertionError();
                }
                c3673g.b.add(abstractC1076Vw2);
                ((Map) map.computeIfAbsent(abstractC1076Vw2, new Function() { // from class: com.android.tools.r8.ir.optimize.q$$ExternalSyntheticLambda12
                    @Override // java.util.function.Function
                    public final Object apply(Object obj) {
                        return C3692q.a((AbstractC1076Vw) obj);
                    }
                })).put(c3161ul0, c3677i);
            }
        }
        for (SW sw : c3161ul0.a0()) {
            boolean z2 = c;
            if (!z2 && !sw.c0().contains(c3161ul0)) {
                throw new AssertionError();
            }
            List<C3161ul0> listC0 = sw.c0();
            List<H5> listS = sw.r.s();
            if (!z2 && listC0.size() != listS.size()) {
                throw new AssertionError();
            }
            C1105Wz c1105Wz = new C1105Wz(16);
            Iterator<C3161ul0> it = listC0.iterator();
            Iterator<H5> it2 = listS.iterator();
            int i = 0;
            while (it.hasNext() && it2.hasNext()) {
                C3161ul0 next2 = it.next();
                H5 next3 = it2.next();
                if (next2 == c3161ul0 && set.contains(next3)) {
                    c1105Wz.add(i);
                }
                i++;
            }
            if (!c1105Wz.isEmpty()) {
                boolean z3 = C3673g.d;
                if (!z3 && !c3673g.a.a0().contains(sw)) {
                    throw new AssertionError();
                }
                if (!z3 && c3673g.c.containsKey(sw)) {
                    throw new AssertionError();
                }
                c3673g.c.put(sw, c1105Wz);
            }
        }
        if (c3673g.b.isEmpty() && c3673g.c.isEmpty()) {
            return C3689n.a;
        }
        if (!C3673g.d && c3673g.b.size() >= c3673g.a.b0().size() && c3673g.c.size() >= c3673g.a.a0().size()) {
            throw new AssertionError();
        }
        return new C3690o(c3673g.b, c3673g.c);
    }

    public static /* synthetic */ boolean a(C3161ul0 c3161ul0, AbstractC1076Vw abstractC1076Vw) {
        return abstractC1076Vw != c3161ul0.c;
    }

    public static /* synthetic */ boolean a(AbstractC1076Vw abstractC1076Vw, AbstractC1076Vw abstractC1076Vw2) {
        return abstractC1076Vw2 != abstractC1076Vw;
    }

    public static /* synthetic */ Set a(C2435mm c2435mm, H5 h5) {
        return (Set) c2435mm.a(h5, AbstractC3424xb0.c());
    }

    public static /* synthetic */ boolean a(AbstractC1076Vw abstractC1076Vw, AbstractC1076Vw abstractC1076Vw2, AbstractC1076Vw abstractC1076Vw3) {
        return abstractC1076Vw3 == abstractC1076Vw || abstractC1076Vw3 == abstractC1076Vw2;
    }

    public static /* synthetic */ Map a(AbstractC1076Vw abstractC1076Vw) {
        return new IdentityHashMap();
    }

    public static boolean a(C3677i c3677i) {
        c3677i.a.getClass();
        return !(r0 instanceof C3687l);
    }

    public static /* synthetic */ void a(C0874Ot c0874Ot, H5 h5, Map map) {
        K5 k5A = h5.a(c0874Ot);
        while (k5A.hasNext() && !map.isEmpty()) {
            List list = (List) map.remove(k5A.next());
            if (list != null) {
                list.forEach(new p3$$ExternalSyntheticLambda1(k5A));
            }
        }
    }

    public final void a(final C0874Ot c0874Ot, C3686k c3686k, final Set set, final IdentityHashMap identityHashMap, final Predicate predicate) {
        c3686k.a(new InterfaceC2604oi0() { // from class: com.android.tools.r8.ir.optimize.q$$ExternalSyntheticLambda7
            @Override // com.android.tools.r8.internal.InterfaceC2604oi0
            public final boolean a(Object obj, Object obj2, Object obj3) {
                return this.f$0.a(predicate, c0874Ot, set, identityHashMap, (AbstractC1076Vw) obj, (C3161ul0) obj2, (C3677i) obj3);
            }
        });
    }

    public final boolean a(Predicate predicate, C0874Ot c0874Ot, Set set, Map map, final AbstractC1076Vw abstractC1076Vw, final C3161ul0 c3161ul0, C3677i c3677i) {
        H5 h5A;
        AbstractC3250vj0 abstractC3250vj0T;
        C3161ul0 c3161ul0A;
        AbstractC1076Vw abstractC1076VwA;
        if (!predicate.test(c3677i)) {
            return false;
        }
        H5 h5B = abstractC1076Vw.b();
        if (abstractC1076Vw.M1()) {
            C2445mu c2445muT = abstractC1076Vw.T();
            if (c2445muT.O2()) {
                if (!C2445mu.k && !c2445muT.P2()) {
                    throw new AssertionError();
                }
                h5A = c2445muT.b(Y6.a(true));
            } else {
                h5A = c2445muT.S2();
            }
        } else {
            H5 h5B2 = abstractC1076Vw.b();
            if (!h5B2.x()) {
                h5A = h5B2;
            } else {
                h5A = C5.a(h5B2);
            }
        }
        AbstractC3675h abstractC3675h = c3677i.a;
        if (c3677i.b.d().e()) {
            c3161ul0A = c0874Ot.a(AbstractC3250vj0.m(), (C0286j0) null);
        } else {
            if (c3677i.b()) {
                abstractC3250vj0T = c3161ul0.t().d().P();
            } else {
                abstractC3250vj0T = c3161ul0.t();
            }
            c3161ul0A = c0874Ot.a(abstractC3250vj0T, c3161ul0.r());
        }
        abstractC3675h.getClass();
        if (abstractC3675h instanceof C3687l) {
            c3161ul0.f(c3161ul0A);
        } else if (abstractC3675h instanceof C3688m) {
            Predicate predicate2 = new Predicate() { // from class: com.android.tools.r8.ir.optimize.q$$ExternalSyntheticLambda13
                @Override // java.util.function.Predicate
                public final boolean test(Object obj) {
                    return C3692q.c(abstractC1076Vw, (AbstractC1076Vw) obj);
                }
            };
            if (c3161ul0 == c3161ul0A) {
                c3161ul0.getClass();
            } else {
                for (AbstractC1076Vw abstractC1076Vw2 : c3161ul0.b0()) {
                    if (predicate2.test(abstractC1076Vw2)) {
                        c3161ul0.b(abstractC1076Vw2);
                        abstractC1076Vw2.a(c3161ul0, c3161ul0A, (Set) null);
                    }
                }
            }
            c3161ul0.e(c3161ul0A);
        } else if (abstractC3675h instanceof C3690o) {
            C3690o c3690oA = abstractC3675h.a();
            c3690oA.b.forEach(new BiConsumer() { // from class: com.android.tools.r8.ir.optimize.q$$ExternalSyntheticLambda14
                @Override // java.util.function.BiConsumer
                public final void accept(Object obj, Object obj2) {
                    C3692q.a(c3161ul0, (SW) obj, (InterfaceC2382mA) obj2);
                }
            });
            c3161ul0.a(c3161ul0A, c3690oA.a, c3690oA.b, null);
        }
        set.addAll(c3161ul0A.a());
        if (c3677i.b.d().e()) {
            abstractC1076VwA = new C2026ig(c3161ul0A, 0L);
        } else {
            abstractC1076VwA = C3571z4.a(c3677i.b, c3161ul0A, c3161ul0, abstractC1076Vw, this.a, c0874Ot.i());
        }
        abstractC1076VwA.b(abstractC1076Vw.getPosition());
        if (h5A != h5B) {
            h5A.a(c0874Ot).add(abstractC1076VwA);
        } else {
            ((List) ((Map) map.computeIfAbsent(h5B, new Function() { // from class: com.android.tools.r8.ir.optimize.q$$ExternalSyntheticLambda15
                @Override // java.util.function.Function
                public final Object apply(Object obj) {
                    return C3692q.a((H5) obj);
                }
            })).computeIfAbsent(abstractC1076Vw, new Function() { // from class: com.android.tools.r8.ir.optimize.q$$ExternalSyntheticLambda1
                @Override // java.util.function.Function
                public final Object apply(Object obj) {
                    return C3692q.b((AbstractC1076Vw) obj);
                }
            })).add(abstractC1076VwA);
        }
        return true;
    }

    public static void a(C3161ul0 c3161ul0, SW sw, InterfaceC2382mA interfaceC2382mA) {
        InterfaceC2472nA it = interfaceC2382mA.iterator();
        while (it.hasNext()) {
            C3161ul0 c3161ul02 = (C3161ul0) sw.s.get(it.q());
            if (c3161ul02 != c3161ul0) {
                if (!c && !c3161ul02.c(new fw$$ExternalSyntheticLambda5())) {
                    throw new AssertionError();
                }
                it.remove();
            }
        }
    }

    public static /* synthetic */ Map a(H5 h5) {
        return new IdentityHashMap();
    }

    public static boolean a(final AbstractC1076Vw abstractC1076Vw, C3161ul0 c3161ul0) {
        return c3161ul0.A() || AC.b(c3161ul0.b0(), new HX() { // from class: com.android.tools.r8.ir.optimize.q$$ExternalSyntheticLambda0
            @Override // com.android.tools.r8.internal.HX
            public final boolean apply(Object obj) {
                return C3692q.b(abstractC1076Vw, (AbstractC1076Vw) obj);
            }
        });
    }
}
