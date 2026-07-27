package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.AbstractC2584oX;
import com.android.tools.r8.internal.AbstractC3424xb0;
import com.android.tools.r8.internal.C0831Nh;
import com.android.tools.r8.internal.C0874Ot;
import com.android.tools.r8.internal.C0996Tg;
import com.android.tools.r8.internal.C1762fq;
import com.android.tools.r8.internal.C2026ig;
import com.android.tools.r8.internal.C2527no;
import com.android.tools.r8.internal.C2824r8;
import com.android.tools.r8.internal.C3080ts;
import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.internal.H5;
import com.android.tools.r8.internal.K5;
import com.android.tools.r8.internal.PX;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.function.Predicate;

/* JADX INFO: renamed from: com.android.tools.r8.ir.optimize.e0, reason: case insensitive filesystem */
/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public class C3667e0 {
    public static final /* synthetic */ boolean a = true;

    public static void a(C0421y<?> c0421y, C0874Ot c0874Ot, com.android.tools.r8.ir.regalloc.b bVar) {
        a(c0874Ot, bVar);
        for (H5 h5 : c0874Ot.d) {
            HashMap map = new HashMap();
            C3659a0 c3659a0 = new C3659a0(bVar);
            K5 k5A = h5.a(c0874Ot);
            while (k5A.hasNext()) {
                AbstractC1076Vw next = k5A.next();
                if (c3659a0.a(next)) {
                    k5A.r();
                } else if (next.d() != null && next.d().T()) {
                    C3161ul0 c3161ul0D = next.d();
                    int i = next.e;
                    if (c3161ul0D.J() && next.A1()) {
                        C2026ig c2026igH = next.H();
                        C3161ul0 c3161ul0D2 = c2026igH.d();
                        c3161ul0D2.getClass();
                        if (c3161ul0D2 instanceof C1762fq ? false : c2026igH.d().j.c(c2026igH.e).j()) {
                            k5A.r();
                        } else {
                            int iA = bVar.a(c3161ul0D, i);
                            C2026ig c2026ig = (C2026ig) map.get(Integer.valueOf(iA));
                            if (c2026ig == null || !c2026ig.b(next)) {
                                map.put(Integer.valueOf(iA), next.H());
                                if (next.I2().b()) {
                                    map.remove(Integer.valueOf(iA + 1));
                                }
                                int i2 = iA - 1;
                                C2026ig c2026ig2 = (C2026ig) map.get(Integer.valueOf(i2));
                                if (c2026ig2 != null && c2026ig2.I2().b()) {
                                    map.remove(Integer.valueOf(i2));
                                }
                            } else {
                                k5A.r();
                            }
                        }
                    } else {
                        int iA2 = bVar.a(c3161ul0D, i);
                        for (int i3 = 0; i3 < c3161ul0D.o.O(); i3++) {
                            map.remove(Integer.valueOf(iA2 + i3));
                        }
                        int i4 = iA2 - 1;
                        C2026ig c2026ig3 = (C2026ig) map.get(Integer.valueOf(i4));
                        if (c2026ig3 != null && c2026ig3.I2().b()) {
                            map.remove(Integer.valueOf(i4));
                        }
                    }
                }
            }
        }
        Z z = new Z(c0874Ot, bVar);
        Set setC = AbstractC3424xb0.c();
        Iterator<H5> it = c0874Ot.d.iterator();
        while (it.hasNext()) {
            a(it.next(), bVar, setC, z);
        }
        c0874Ot.d.removeAll(setC);
        a(c0874Ot, bVar, 0);
        if (a) {
            return;
        }
        c0874Ot.b((C0421y) c0421y, false);
    }

    public static /* synthetic */ List b(C2527no c2527no) {
        return new ArrayList();
    }

    public static void a(final H5 h5, final com.android.tools.r8.ir.regalloc.f fVar, Set set, Z z) {
        if (set.contains(h5) || !a(h5)) {
            return;
        }
        List listN = h5.n();
        while (true) {
            H5 h52 = (H5) listN.get(0);
            Iterator it = listN.iterator();
            while (it.hasNext()) {
                if (((H5) it.next()).f.isEmpty()) {
                    if (!a && !set.containsAll(listN)) {
                        throw new AssertionError();
                    }
                    return;
                }
            }
            final AbstractC1076Vw abstractC1076Vw = (AbstractC1076Vw) h52.f.get(0);
            for (int i = 1; i < listN.size(); i++) {
                if (!z.b(abstractC1076Vw, (AbstractC1076Vw) ((H5) listN.get(i)).f.get(0))) {
                    return;
                }
            }
            if (abstractC1076Vw.i()) {
                if (h5.x()) {
                    return;
                }
                Iterator it2 = listN.iterator();
                while (it2.hasNext()) {
                    if (((H5) it2.next()).x()) {
                        return;
                    }
                }
            }
            if (abstractC1076Vw.d() != null && abstractC1076Vw.d().T()) {
                final int iA = fVar.a(abstractC1076Vw.d(), abstractC1076Vw.e);
                if (!h5.h().c.stream().allMatch(new Predicate() { // from class: com.android.tools.r8.ir.optimize.e0$$ExternalSyntheticLambda0
                    @Override // java.util.function.Predicate
                    public final boolean test(Object obj) {
                        return C3667e0.a(fVar, h5, abstractC1076Vw, iA, (C3161ul0) obj);
                    }
                })) {
                    return;
                }
            }
            AbstractC2584oX position = abstractC1076Vw.getPosition();
            AbstractC2584oX position2 = h5.h().getPosition();
            position.getClass();
            if (!com.android.tools.r8.utils.structural.k.a(position, position2) && (!h5.h().getPosition().n() || h5.h().T0().isEmpty())) {
                return;
            }
            Iterator it3 = listN.iterator();
            while (it3.hasNext()) {
                ((H5) it3.next()).k().removeFirst();
            }
            if (abstractC1076Vw.d2()) {
                LinkedList<AbstractC1076Vw> linkedListK = h5.k();
                linkedListK.removeLast();
                linkedListK.add(abstractC1076Vw);
                abstractC1076Vw.a(h5);
                ArrayList<H5> arrayList = new ArrayList(listN);
                Iterator it4 = h5.b.iterator();
                while (it4.hasNext()) {
                    ((H5) it4.next()).l().remove(h5);
                }
                h5.m().clear();
                Iterator it5 = h52.n().iterator();
                while (it5.hasNext()) {
                    h5.h((H5) it5.next());
                }
                for (H5 h53 : arrayList) {
                    Iterator it6 = h53.b.iterator();
                    while (it6.hasNext()) {
                        ((H5) it6.next()).l().remove(h53);
                    }
                    h53.m().clear();
                }
                set.addAll(arrayList);
                if (!a(h5)) {
                    return;
                }
            } else {
                h5.k().listIterator(h5.k().size() - 1).add(abstractC1076Vw);
                abstractC1076Vw.a(h5);
                if (abstractC1076Vw instanceof C0831Nh) {
                    C0831Nh c0831NhK = abstractC1076Vw.K();
                    Iterator it7 = listN.iterator();
                    while (it7.hasNext()) {
                        c0831NhK.a(((H5) it7.next()).a);
                    }
                }
            }
        }
    }

    public static boolean a(com.android.tools.r8.ir.regalloc.f fVar, H5 h5, AbstractC1076Vw abstractC1076Vw, int i, C3161ul0 c3161ul0) {
        int iA = fVar.a(c3161ul0, h5.h().e);
        for (int i2 = 0; i2 < abstractC1076Vw.d().o.O(); i2++) {
            for (int i3 = 0; i3 < c3161ul0.o.O(); i3++) {
                if (i + i2 == iA + i3) {
                    return false;
                }
            }
        }
        return true;
    }

    public static boolean a(H5 h5) {
        List listN = h5.n();
        if (listN.size() <= 1) {
            return false;
        }
        Iterator it = listN.iterator();
        while (it.hasNext()) {
            if (((H5) it.next()).s().size() != 1) {
                return false;
            }
        }
        H5 h52 = (H5) listN.get(0);
        for (int i = 1; i < listN.size(); i++) {
            if (!Objects.equals(h52.a, ((H5) listN.get(i)).a)) {
                return false;
            }
        }
        return true;
    }

    /* JADX WARN: Code restructure failed: missing block: B:105:0x026b, code lost:
    
        throw new java.lang.AssertionError();
     */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public static void a(com.android.tools.r8.internal.C0874Ot r22, com.android.tools.r8.ir.regalloc.f r23, int r24) {
        /*
            Method dump skipped, instruction units count: 976
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.android.tools.r8.ir.optimize.C3667e0.a(com.android.tools.r8.internal.Ot, com.android.tools.r8.ir.regalloc.f, int):void");
    }

    public static /* synthetic */ List a(C2527no c2527no) {
        return new ArrayList();
    }

    public static void a(C0874Ot c0874Ot, com.android.tools.r8.ir.regalloc.f fVar) {
        boolean z;
        C3698s c3698s = new C3698s(c0874Ot, fVar);
        do {
            z = false;
            for (H5 h5 : c0874Ot.d) {
                HashMap map = new HashMap();
                for (int i = 0; i < h5.s().size(); i++) {
                    H5 h52 = h5.s().get(i);
                    if (h52.k().size() != 1) {
                        C2527no c2527no = new C2527no(c3698s, h52);
                        if (map.containsKey(c2527no)) {
                            H5 h53 = h5.s().get(((Integer) map.get(c2527no)).intValue());
                            if (!a && fVar.c().Z0 && !Objects.equals(h52.r(), h53.r())) {
                                throw new AssertionError();
                            }
                            fVar.b(h53, h52);
                            h52.e = C2824r8.e;
                            h52.k().clear();
                            c3698s.c[h52.o()] = -1;
                            for (H5 h54 : h52.t()) {
                                h54.getClass();
                                h54.a(h52, (C3658a) null, C0996Tg.b(), PX.c);
                            }
                            h52.m().clear();
                            h52.m().add(h53);
                            if (!a && h53.s().contains(h52)) {
                                throw new AssertionError();
                            }
                            h53.l().add(h52);
                            C3080ts c3080ts = new C3080ts();
                            c3080ts.a(h52);
                            c3080ts.b(h53.r());
                            h52.k().add(c3080ts);
                            z = true;
                        } else {
                            map.put(c2527no, Integer.valueOf(i));
                        }
                    }
                }
            }
        } while (z);
    }
}
