package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.C0286j0;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.H2;
import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.AbstractC2584oX;
import com.android.tools.r8.internal.C0756Kt;
import com.android.tools.r8.internal.C0800Mh;
import com.android.tools.r8.internal.C0874Ot;
import com.android.tools.r8.internal.C2806qz;
import com.android.tools.r8.internal.C2881rj0;
import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.internal.C3177uz;
import com.android.tools.r8.internal.C3465xz;
import com.android.tools.r8.internal.C3571z4;
import com.android.tools.r8.internal.C3655zx;
import com.android.tools.r8.internal.Ge0;
import com.android.tools.r8.internal.H5;
import com.android.tools.r8.internal.InterfaceC1160Yw;
import com.android.tools.r8.internal.InterfaceC2356lz;
import com.android.tools.r8.internal.K5;
import com.android.tools.r8.internal.Nk0;
import com.android.tools.r8.internal.fw$$ExternalSyntheticLambda5;
import java.util.Iterator;
import java.util.Set;

/* JADX INFO: renamed from: com.android.tools.r8.ir.optimize.x, reason: case insensitive filesystem */
/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public class C3714x {
    public static final /* synthetic */ boolean b = true;
    public final C0421y a;

    public C3714x(C0421y c0421y) {
        this.a = c0421y;
    }

    public static void a(C0421y<?> c0421y, C0874Ot c0874Ot) {
        C3658a c3658a = new C3658a();
        InterfaceC1160Yw interfaceC1160YwR = c0874Ot.r();
        boolean z = false;
        while (interfaceC1160YwR.hasNext()) {
            AbstractC1076Vw next = interfaceC1160YwR.next();
            next.getClass();
            if (next instanceof C3571z4) {
                C3571z4 c3571z4Z = next.z();
                C3161ul0 c3161ul0L2 = c3571z4Z.L2();
                C3161ul0 c3161ul0D = c3571z4Z.d();
                c3658a.b.addAll(c3161ul0D.a());
                z |= c3161ul0D.V() > 0;
                c3161ul0D.f(c3161ul0L2);
                interfaceC1160YwR.remove();
            }
        }
        if (z) {
            c0874Ot.a((C0756Kt) null, c3658a);
        }
        if (!c3658a.b.isEmpty()) {
            new C2881rj0(c0421y, c0874Ot, false).a(c3658a, 2);
        }
        c0874Ot.y();
        boolean z2 = b;
        if (!z2 && !Ge0.a(c0874Ot.s()).noneMatch(new fw$$ExternalSyntheticLambda5())) {
            throw new AssertionError();
        }
        if (!z2 && !c0874Ot.b(c0421y)) {
            throw new AssertionError();
        }
    }

    public static void a(AbstractC1076Vw abstractC1076Vw, InterfaceC1160Yw interfaceC1160Yw, C3161ul0 c3161ul0, C3161ul0 c3161ul02) {
        if (c3161ul02.y() && c3161ul02.r() != c3161ul0.r()) {
            interfaceC1160Yw.e(new C0800Mh(c3161ul02, c3161ul0));
            return;
        }
        if (c3161ul02.y()) {
            if (!b && c3161ul02.r() != c3161ul0.r()) {
                throw new AssertionError();
            }
            C0286j0 c0286j0R = c3161ul02.r();
            Set set = abstractC1076Vw.f;
            if (set != null) {
                Iterator it = set.iterator();
                while (true) {
                    if (!it.hasNext()) {
                        break;
                    }
                    C3161ul0 c3161ul03 = (C3161ul0) it.next();
                    if (c3161ul03.y() && c3161ul03.r() == c0286j0R) {
                        it.remove();
                        c3161ul03.c(abstractC1076Vw);
                        break;
                    }
                }
            }
        }
        c3161ul02.f(c3161ul0);
        interfaceC1160Yw.p();
    }

    public final void a(C0874Ot c0874Ot) {
        AbstractC1076Vw abstractC1076Vw;
        Iterator<H5> it = c0874Ot.d.iterator();
        while (it.hasNext()) {
            K5 k5A = it.next().a(c0874Ot);
            while (k5A.hasNext()) {
                AbstractC1076Vw abstractC1076VwI = k5A.i();
                AbstractC1076Vw next = k5A.next();
                next.getClass();
                if (next instanceof C0800Mh) {
                    if (!b && next.c.size() != 1) {
                        throw new AssertionError();
                    }
                    C3161ul0 c3161ul0 = (C3161ul0) next.c.get(0);
                    C0286j0 c0286j0R = next.d().r();
                    H2 h2 = c0286j0R.b;
                    if (!c3161ul0.y() && c3161ul0.U() == 1 && (abstractC1076Vw = c3161ul0.c) != null && abstractC1076Vw.b() == next.b()) {
                        if (!abstractC1076Vw.getPosition().n() && !next.getPosition().n()) {
                            AbstractC2584oX position = abstractC1076Vw.getPosition();
                            AbstractC2584oX position2 = next.getPosition();
                            position.getClass();
                            if (!com.android.tools.r8.utils.structural.k.a(position, position2)) {
                                continue;
                            }
                        }
                        C3161ul0 c3161ul02 = null;
                        AbstractC2584oX position3 = null;
                        for (AbstractC1076Vw abstractC1076Vw2 : abstractC1076Vw.b().a(abstractC1076Vw)) {
                            if (position3 == null) {
                                if (!abstractC1076Vw2.getPosition().n()) {
                                    position3 = abstractC1076Vw2.getPosition();
                                }
                            } else if (abstractC1076Vw2.getPosition().n() || com.android.tools.r8.utils.structural.k.a(position3, abstractC1076Vw2.getPosition())) {
                            }
                            if (abstractC1076Vw2 == next) {
                                c3161ul0.a(c0286j0R);
                                next.d().f(c3161ul0);
                                Set set = next.f;
                                if (set != null) {
                                    Iterator it2 = set.iterator();
                                    while (true) {
                                        if (!it2.hasNext()) {
                                            break;
                                        }
                                        C3161ul0 c3161ul03 = (C3161ul0) it2.next();
                                        if (c3161ul03.y() && c3161ul03.r() == c0286j0R) {
                                            it2.remove();
                                            c3161ul03.c(next);
                                            c3161ul02 = c3161ul03;
                                            break;
                                        }
                                    }
                                }
                                if (c3161ul02 != null) {
                                    c3161ul02.a(c3161ul0.c);
                                }
                                if (abstractC1076VwI != null && (abstractC1076VwI.d() == null || !abstractC1076VwI.d().y() || !next.T0().contains(abstractC1076VwI.d()))) {
                                    next.c(abstractC1076VwI);
                                }
                                k5A.p();
                            } else if (abstractC1076Vw2.d() == null || !abstractC1076Vw2.d().y() || abstractC1076Vw2.d().r().b != h2) {
                            }
                        }
                        throw new Nk0();
                    }
                }
            }
        }
        if (!b && !c0874Ot.b(this.a)) {
            throw new AssertionError();
        }
    }

    public static void a(C3655zx c3655zx, C3465xz c3465xz) {
        if (c3655zx.isEmpty()) {
            return;
        }
        C3465xz c3465xz2 = new C3465xz(c3465xz);
        c3465xz.clear();
        C2806qz c2806qz = new C2806qz(((C3177uz) c3465xz2.c()).b);
        while (c2806qz.hasNext()) {
            InterfaceC2356lz interfaceC2356lz = (InterfaceC2356lz) c2806qz.next();
            int iA = interfaceC2356lz.a();
            c3465xz.a(((Integer) c3655zx.getOrDefault(Integer.valueOf(iA), Integer.valueOf(iA))).intValue(), (C0286j0) interfaceC2356lz.getValue());
        }
    }
}
