package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.C0874Ot;
import com.android.tools.r8.internal.C0996Tg;
import com.android.tools.r8.internal.C2554o7;
import com.android.tools.r8.internal.C3010t7;
import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.internal.C3409xP;
import com.android.tools.r8.internal.Cl0;
import com.android.tools.r8.internal.Fh0;
import com.android.tools.r8.internal.H5;
import com.android.tools.r8.internal.SW;
import java.util.ArrayDeque;
import java.util.Iterator;
import java.util.Queue;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class F {
    public static final /* synthetic */ boolean b = true;
    public final C0421y a;

    public F(C0421y c0421y) {
        this.a = c0421y;
    }

    public final void a(C0874Ot c0874Ot, Fh0 fh0) {
        fh0.a("Remove dead code");
        new C3409xP(this.a).a(c0874Ot, fh0);
        C3010t7 c3010t7 = new C3010t7(this.a);
        ArrayDeque arrayDeque = new ArrayDeque();
        while (true) {
            C3658a c3658a = new C3658a();
            Cl0 cl0 = new Cl0(this.a, c0874Ot);
            arrayDeque.addAll(c0874Ot.D());
            while (!arrayDeque.isEmpty()) {
                H5 h5 = (H5) arrayDeque.removeLast();
                a(arrayDeque, c0874Ot, h5, c3658a, cl0);
                Iterator<SW> it = h5.q().iterator();
                while (it.hasNext()) {
                    SW next = it.next();
                    if (cl0.a(next)) {
                        it.remove();
                        for (C3161ul0 c3161ul0 : next.c0()) {
                            c3161ul0.f.remove(next);
                            c3161ul0.g = null;
                            a(arrayDeque, c3161ul0);
                        }
                    } else if (next.d0()) {
                        it.remove();
                        next.g0();
                    }
                }
            }
            c3658a.a(this.a, c0874Ot, C0996Tg.b());
            C2554o7 c2554o7C = c3010t7.c(c0874Ot);
            c2554o7C.getClass();
            if (!c2554o7C.e && !a(c0874Ot)) {
                break;
            }
        }
        c0874Ot.y();
        boolean z = b;
        if (!z && !c0874Ot.b(this.a)) {
            throw new AssertionError();
        }
        if (!z) {
            b(c0874Ot);
        }
        fh0.b();
    }

    public final void b(C0874Ot c0874Ot) {
        boolean z = b;
        if (!z && !new C3409xP(this.a).a(c0874Ot, Fh0.a()).a().a()) {
            throw new AssertionError();
        }
        if (!z && a(c0874Ot)) {
            throw new AssertionError();
        }
        Cl0 cl0 = new Cl0(this.a, c0874Ot);
        for (H5 h5 : c0874Ot.d) {
            if (!b && cl0.a(h5)) {
                throw new AssertionError();
            }
            for (AbstractC1076Vw abstractC1076Vw : h5.k()) {
                boolean z2 = b;
                if (!z2 && abstractC1076Vw.S1() && abstractC1076Vw.d1() && !abstractC1076Vw.d().v()) {
                    throw new AssertionError();
                }
                if (!z2) {
                    E eA = abstractC1076Vw.a(this.a, c0874Ot);
                    eA.getClass();
                    if ((eA instanceof B) && (!abstractC1076Vw.d1() || cl0.a(abstractC1076Vw.d()))) {
                        throw new AssertionError();
                    }
                }
            }
        }
    }

    public static void a(Queue queue, C3161ul0 c3161ul0) {
        H5 h5B;
        if (c3161ul0.l()) {
            h5B = c3161ul0.p().r;
        } else if (!c3161ul0.c.a1()) {
            h5B = null;
        } else {
            h5B = c3161ul0.c.b();
        }
        if (h5B != null) {
            ((ArrayDeque) queue).add(h5B);
        }
    }

    /* JADX WARN: Removed duplicated region for block: B:30:0x0082  */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public final void a(java.util.Queue r6, com.android.tools.r8.internal.C0874Ot r7, com.android.tools.r8.internal.H5 r8, com.android.tools.r8.ir.optimize.C3658a r9, com.android.tools.r8.internal.Cl0 r10) {
        /*
            Method dump skipped, instruction units count: 470
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.android.tools.r8.ir.optimize.F.a(java.util.Queue, com.android.tools.r8.internal.Ot, com.android.tools.r8.internal.H5, com.android.tools.r8.ir.optimize.a, com.android.tools.r8.internal.Cl0):void");
    }

    /* JADX WARN: Code restructure failed: missing block: B:44:0x00d8, code lost:
    
        r7 = r7 + 1;
     */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public final boolean a(com.android.tools.r8.internal.C0874Ot r14) {
        /*
            Method dump skipped, instruction units count: 429
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.android.tools.r8.ir.optimize.F.a(com.android.tools.r8.internal.Ot):boolean");
    }
}
