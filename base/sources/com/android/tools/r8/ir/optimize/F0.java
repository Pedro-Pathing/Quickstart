package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.B1;
import com.android.tools.r8.graph.C0257g1;
import com.android.tools.r8.graph.C0309l1;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.D2;
import com.android.tools.r8.graph.H2;
import com.android.tools.r8.graph.I2;
import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.AbstractC2575oO;
import com.android.tools.r8.internal.C0874Ot;
import com.android.tools.r8.internal.C1284az;
import com.android.tools.r8.internal.C2929sC;
import com.android.tools.r8.internal.C3147ue0;
import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.shaking.C3877i;
import java.util.IdentityHashMap;
import java.util.Iterator;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Predicate;
import java.util.stream.Collectors;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class F0 {
    public final C0421y a;
    public final H2 b;
    public final H2 c;
    public final I2 d;
    public final IdentityHashMap e = new IdentityHashMap();

    public F0(C0421y c0421y) {
        this.a = c0421y;
        B1 b1A = c0421y.a();
        this.b = b1A.c("$SwitchMap$");
        this.c = b1A.c("$EnumSwitchMapping$");
        this.d = b1A.K1;
    }

    public final C3877i a() {
        Iterator<D2> it = ((C3877i) this.a.g()).d().iterator();
        while (it.hasNext()) {
            a(it.next());
        }
        if (this.e.isEmpty()) {
            return (C3877i) this.a.g();
        }
        C3877i c3877i = (C3877i) this.a.g();
        IdentityHashMap identityHashMap = this.e;
        boolean z = C3877i.J;
        if (!z) {
            c3877i.c();
        }
        if (z) {
            c3877i.getClass();
        } else if (!c3877i.G.isEmpty()) {
            throw new AssertionError();
        }
        return new C3877i(c3877i, identityHashMap);
    }

    public final void a(D2 d2) {
        if (d2.f.p() && d2.f1()) {
            List list = (List) d2.D1().stream().filter(new Predicate() { // from class: com.android.tools.r8.ir.optimize.F0$$ExternalSyntheticLambda0
                @Override // java.util.function.Predicate
                public final boolean test(Object obj) {
                    return this.f$0.a((C0257g1) obj);
                }
            }).collect(Collectors.toList());
            if (list.isEmpty()) {
                return;
            }
            final C0874Ot c0874OtA = d2.i(d2.O0()).a(this.a, AbstractC2575oO.e());
            list.forEach(new Consumer() { // from class: com.android.tools.r8.ir.optimize.F0$$ExternalSyntheticLambda1
                @Override // java.util.function.Consumer
                public final void accept(Object obj) {
                    this.f$0.b(c0874OtA, (C0257g1) obj);
                }
            });
        }
    }

    /* JADX INFO: renamed from: a, reason: merged with bridge method [inline-methods] */
    public final void b(C0874Ot c0874Ot, C0257g1 c0257g1) {
        AbstractC1076Vw abstractC1076Vw;
        final C0309l1 reference = c0257g1.getReference();
        C1284az c1284az = new C1284az();
        for (AbstractC1076Vw abstractC1076Vw2 : c0874Ot.b(new Predicate() { // from class: com.android.tools.r8.ir.optimize.F0$$ExternalSyntheticLambda2
            @Override // java.util.function.Predicate
            public final boolean test(Object obj) {
                return F0.a(reference, (AbstractC1076Vw) obj);
            }
        })) {
            abstractC1076Vw2.getClass();
            for (AbstractC1076Vw abstractC1076Vw3 : (abstractC1076Vw2 instanceof C3147ue0 ? abstractC1076Vw2.d() : abstractC1076Vw2.J0().value()).b0()) {
                if (abstractC1076Vw3.q1()) {
                    AbstractC1076Vw abstractC1076Vw4 = abstractC1076Vw3.y().value().c;
                    if (abstractC1076Vw4 == null || !abstractC1076Vw4.A1()) {
                        return;
                    }
                    int iN2 = abstractC1076Vw4.H().N2();
                    AbstractC1076Vw abstractC1076Vw5 = ((C3161ul0) abstractC1076Vw3.y().c.get(1)).c;
                    if (abstractC1076Vw5 == null || !abstractC1076Vw5.c2()) {
                        return;
                    }
                    C2929sC c2929sCI0 = abstractC1076Vw5.i0();
                    com.android.tools.r8.graph.E0 e0D = this.a.d(c2929sCI0.U2().f);
                    if (e0D == null) {
                        return;
                    }
                    if ((!e0D.f.K() && e0D.e != this.a.a().f2) || (abstractC1076Vw = ((C3161ul0) c2929sCI0.c.get(0)).c) == null || !(abstractC1076Vw instanceof C3147ue0)) {
                        return;
                    }
                    C0309l1 field = abstractC1076Vw.I0().getField();
                    com.android.tools.r8.graph.E0 e0D2 = this.a.d(field.f);
                    if (e0D2 == null || !e0D2.f.K() || c1284az.a(iN2, field) != null) {
                        return;
                    }
                } else if (abstractC1076Vw3 != abstractC1076Vw2) {
                    return;
                }
            }
        }
        this.e.put(reference, c1284az);
    }

    public static boolean a(C0309l1 c0309l1, AbstractC1076Vw abstractC1076Vw) {
        abstractC1076Vw.getClass();
        return ((abstractC1076Vw instanceof C3147ue0) || abstractC1076Vw.e()) && abstractC1076Vw.Q().getField() == c0309l1;
    }

    /* JADX WARN: Removed duplicated region for block: B:8:0x002c  */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public final boolean a(com.android.tools.r8.graph.C0257g1 r3) {
        /*
            r2 = this;
            com.android.tools.r8.graph.r2 r0 = r3.getReference()
            com.android.tools.r8.graph.l1 r0 = (com.android.tools.r8.graph.C0309l1) r0
            com.android.tools.r8.graph.e3 r3 = r3.g
            boolean r3 = r3.p()
            if (r3 == 0) goto L34
            com.android.tools.r8.graph.H2 r3 = r0.g
            com.android.tools.r8.graph.H2 r1 = r2.b
            r3.getClass()
            byte[] r1 = r1.f
            boolean r3 = r3.b(r1)
            if (r3 != 0) goto L2c
            com.android.tools.r8.graph.H2 r3 = r0.g
            com.android.tools.r8.graph.H2 r1 = r2.c
            r3.getClass()
            byte[] r1 = r1.f
            boolean r3 = r3.b(r1)
            if (r3 == 0) goto L34
        L2c:
            com.android.tools.r8.graph.I2 r3 = r0.i
            com.android.tools.r8.graph.I2 r0 = r2.d
            if (r3 != r0) goto L34
            r3 = 1
            goto L35
        L34:
            r3 = 0
        L35:
            return r3
        */
        throw new UnsupportedOperationException("Method not decompiled: com.android.tools.r8.ir.optimize.F0.a(com.android.tools.r8.graph.g1):boolean");
    }
}
