package com.android.tools.r8.ir.regalloc;

import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.AbstractC2584oX;
import com.android.tools.r8.internal.AbstractC3250vj0;
import com.android.tools.r8.internal.C0874Ot;
import com.android.tools.r8.internal.C2867rd;
import com.android.tools.r8.internal.C2947sS;
import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.internal.C3200vB;
import com.android.tools.r8.internal.C3214vP;
import com.android.tools.r8.internal.EnumC3471y2;
import com.android.tools.r8.internal.H5;
import com.android.tools.r8.internal.K5;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.Set;
import java.util.function.Function;
import java.util.function.Predicate;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class n {
    public static final /* synthetic */ boolean i = true;
    public final C0874Ot d;
    public final b e;
    public final C2867rd f;
    public final HashMap a = new HashMap();
    public final HashMap b = new HashMap();
    public final HashMap c = new HashMap();
    public final HashMap g = new HashMap();
    public int h = 0;

    public n(C0421y c0421y, C0874Ot c0874Ot, b bVar) {
        this.e = bVar;
        this.d = c0874Ot;
        C2947sS c2947sSH = C2947sS.h();
        boolean z = AbstractC3250vj0.a;
        this.f = AbstractC3250vj0.a(c0421y.a().a2, c2947sSH, (C0421y<?>) c0421y).b();
        for (H5 h5 : c0874Ot.d) {
            this.g.put(Integer.valueOf(((AbstractC1076Vw) h5.f.get(0)).e), h5);
        }
    }

    public final AbstractC3250vj0 a(c cVar, c cVar2) {
        AbstractC3250vj0 abstractC3250vj0T = cVar.b.t();
        AbstractC3250vj0 abstractC3250vj0T2 = cVar2.b.t();
        if (!abstractC3250vj0T.I() && !abstractC3250vj0T2.I()) {
            if (i || abstractC3250vj0T == abstractC3250vj0T2) {
                return abstractC3250vj0T;
            }
            throw new AssertionError();
        }
        boolean z = i;
        if (!z && !abstractC3250vj0T2.I() && !abstractC3250vj0T2.K()) {
            throw new AssertionError();
        }
        if (z || abstractC3250vj0T.I() || abstractC3250vj0T.K()) {
            return this.f;
        }
        throw new AssertionError();
    }

    public final void b(int i2, c cVar, c cVar2) {
        if (!i && i2 % 2 != 1) {
            throw new AssertionError();
        }
        ((Set) this.b.computeIfAbsent(Integer.valueOf(i2), new Function() { // from class: com.android.tools.r8.ir.regalloc.n$$ExternalSyntheticLambda3
            @Override // java.util.function.Function
            public final Object apply(Object obj) {
                return n.b((Integer) obj);
            }
        })).add(new m(a(cVar, cVar2), cVar, cVar2));
    }

    public final void c(int i2, c cVar, c cVar2) {
        if (!i && i2 % 2 != 1) {
            throw new AssertionError();
        }
        m mVar = new m(a(cVar, cVar2), cVar, cVar2);
        int iC = mVar.b.c();
        int iC2 = cVar.c();
        if (iC <= iC2) {
            c cVar3 = mVar.b;
            if (!c.s && iC2 < cVar3.e.q) {
                throw new AssertionError();
            }
            cVar3.e.q = iC2;
        } else {
            if (!c.s && iC < cVar.e.q) {
                throw new AssertionError();
            }
            cVar.e.q = iC;
        }
        ((Set) this.c.computeIfAbsent(Integer.valueOf(i2), new Function() { // from class: com.android.tools.r8.ir.regalloc.n$$ExternalSyntheticLambda2
            @Override // java.util.function.Function
            public final Object apply(Object obj) {
                return n.c((Integer) obj);
            }
        })).add(mVar);
    }

    public static /* synthetic */ Set b(Integer num) {
        return new LinkedHashSet();
    }

    public static /* synthetic */ Set c(Integer num) {
        return new LinkedHashSet();
    }

    public final void a(int i2, c cVar, c cVar2) {
        if (!i && i2 % 2 != 1) {
            throw new AssertionError();
        }
        ((Set) this.a.computeIfAbsent(Integer.valueOf(i2), new Function() { // from class: com.android.tools.r8.ir.regalloc.n$$ExternalSyntheticLambda5
            @Override // java.util.function.Function
            public final Object apply(Object obj) {
                return n.a((Integer) obj);
            }
        })).add(new m(a(cVar, cVar2), cVar, cVar2));
    }

    public static /* synthetic */ Set a(Integer num) {
        return new LinkedHashSet();
    }

    public final int a(int i2) {
        for (H5 h5 : this.d.d) {
            K5 k5A = h5.a(this.d);
            if (h5 == this.d.j()) {
                while (k5A.hasNext() && k5A.m().l1()) {
                    k5A.next();
                }
                for (C3161ul0 c3161ul0 = this.e.e; c3161ul0 != null; c3161ul0 = c3161ul0.h) {
                    AbstractC1076Vw abstractC1076Vw = c3161ul0.c;
                    if (a(abstractC1076Vw)) {
                        a(i2, abstractC1076Vw, k5A);
                    }
                }
            }
            while (true) {
                AbstractC1076Vw abstractC1076Vw2 = (AbstractC1076Vw) k5A.a(new Predicate() { // from class: com.android.tools.r8.ir.regalloc.n$$ExternalSyntheticLambda1
                    @Override // java.util.function.Predicate
                    public final boolean test(Object obj) {
                        return this.f$0.a((AbstractC1076Vw) obj);
                    }
                });
                if (abstractC1076Vw2 == null) {
                    break;
                }
                if (!(abstractC1076Vw2 instanceof C3214vP)) {
                    k5A.previous();
                }
                a(i2, abstractC1076Vw2, k5A);
            }
        }
        return this.h;
    }

    public final void a(final Set set, final Set set2, final Set set3) {
        set.removeIf(new Predicate() { // from class: com.android.tools.r8.ir.regalloc.n$$ExternalSyntheticLambda4
            @Override // java.util.function.Predicate
            public final boolean test(Object obj) {
                return this.f$0.a(set2, set, set3, (m) obj);
            }
        });
    }

    public final boolean a(Set set, Set set2, Set set3, m mVar) {
        m mVar2;
        m mVar3;
        m mVar4;
        c cVar = mVar.c;
        Iterator it = set.iterator();
        while (true) {
            mVar2 = null;
            if (!it.hasNext()) {
                mVar3 = null;
                break;
            }
            mVar3 = (m) it.next();
            if (mVar3.b == cVar) {
                break;
            }
        }
        int i2 = mVar.b.l;
        int iO = mVar.a.O();
        Iterator it2 = set2.iterator();
        loop1: while (true) {
            if (!it2.hasNext()) {
                mVar4 = null;
                break;
            }
            mVar4 = (m) it2.next();
            int i3 = mVar4.c.l;
            int iO2 = mVar4.a.O();
            for (int i4 = 0; i4 < iO; i4++) {
                for (int i5 = 0; i5 < iO2; i5++) {
                    if (i3 + i5 == i2 + i4) {
                        break loop1;
                    }
                }
            }
        }
        c cVar2 = mVar.c;
        Iterator it3 = set3.iterator();
        while (true) {
            if (!it3.hasNext()) {
                break;
            }
            m mVar5 = (m) it3.next();
            if (mVar5.b == cVar2) {
                mVar2 = mVar5;
                break;
            }
        }
        if (mVar3 == null || mVar4 != null || mVar2 != null) {
            return false;
        }
        mVar3.b = mVar.b;
        return true;
    }

    /* JADX WARN: Removed duplicated region for block: B:14:0x002d  */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public final void a(int r8, com.android.tools.r8.internal.AbstractC1076Vw r9, com.android.tools.r8.internal.K5 r10) {
        /*
            r7 = this;
            boolean r0 = com.android.tools.r8.ir.regalloc.n.i
            if (r0 != 0) goto L11
            boolean r1 = r7.a(r9)
            if (r1 == 0) goto Lb
            goto L11
        Lb:
            java.lang.AssertionError r8 = new java.lang.AssertionError
            r8.<init>()
            throw r8
        L11:
            int r1 = r9.e
            boolean r2 = r10.hasPrevious()
            if (r2 == 0) goto L2d
            com.android.tools.r8.internal.Vw r2 = r10.i()
            r2.getClass()
            boolean r2 = r2 instanceof com.android.tools.r8.internal.C3214vP
            if (r2 == 0) goto L2d
            com.android.tools.r8.internal.Vw r2 = r10.i()
            com.android.tools.r8.internal.oX r2 = r2.getPosition()
            goto L62
        L2d:
            com.android.tools.r8.internal.Vw r2 = r10.m()
            if (r0 != 0) goto L44
            int r3 = r2.e
            if (r3 == r1) goto L44
            boolean r3 = r9.l1()
            if (r3 == 0) goto L3e
            goto L44
        L3e:
            java.lang.AssertionError r8 = new java.lang.AssertionError
            r8.<init>()
            throw r8
        L44:
            com.android.tools.r8.internal.oX r3 = r2.getPosition()
            boolean r4 = r3.n()
            if (r4 == 0) goto L61
            boolean r4 = r2.L1()
            if (r4 == 0) goto L61
            com.android.tools.r8.internal.ts r2 = r2.S()
            com.android.tools.r8.internal.H5 r2 = r2.L2()
            com.android.tools.r8.internal.oX r2 = r2.r()
            goto L62
        L61:
            r2 = r3
        L62:
            java.util.HashMap r3 = r7.a
            int r1 = r1 + (-1)
            java.lang.Integer r4 = java.lang.Integer.valueOf(r1)
            java.util.Set r5 = java.util.Collections.emptySet()
            java.lang.Object r3 = com.android.tools.r8.internal.JM.a(r3, r4, r5)
            java.util.Set r3 = (java.util.Set) r3
            r7.a(r3)
            java.util.HashMap r4 = r7.b
            java.lang.Integer r5 = java.lang.Integer.valueOf(r1)
            java.util.Set r6 = java.util.Collections.emptySet()
            java.lang.Object r4 = com.android.tools.r8.internal.JM.a(r4, r5, r6)
            java.util.Set r4 = (java.util.Set) r4
            r7.a(r4)
            java.util.HashMap r5 = r7.c
            java.lang.Integer r1 = java.lang.Integer.valueOf(r1)
            java.util.Set r6 = java.util.Collections.emptySet()
            java.lang.Object r1 = com.android.tools.r8.internal.JM.a(r5, r1, r6)
            java.util.Set r1 = (java.util.Set) r1
            r7.a(r3, r4, r1)
            boolean r5 = r4.isEmpty()
            if (r5 == 0) goto La5
            r4 = r1
            goto La8
        La5:
            r4.addAll(r1)
        La8:
            r7.a(r8, r3, r10, r2)
            r7.a(r8, r4, r10, r2)
            if (r0 != 0) goto Lbd
            boolean r8 = r7.a(r9)
            if (r8 != 0) goto Lb7
            goto Lbd
        Lb7:
            java.lang.AssertionError r8 = new java.lang.AssertionError
            r8.<init>()
            throw r8
        Lbd:
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: com.android.tools.r8.ir.regalloc.n.a(int, com.android.tools.r8.internal.Vw, com.android.tools.r8.internal.K5):void");
    }

    public final void a(Set set) {
        set.removeIf(new Predicate() { // from class: com.android.tools.r8.ir.regalloc.n$$ExternalSyntheticLambda0
            @Override // java.util.function.Predicate
            public final boolean test(Object obj) {
                return this.f$0.a((m) obj);
            }
        });
    }

    public final boolean a(m mVar) {
        c cVar = mVar.c;
        return cVar.l < this.e.c && cVar.h();
    }

    public final void a(int i2, Set set, K5 k5, AbstractC2584oX abstractC2584oX) {
        if (set.isEmpty()) {
            return;
        }
        h hVar = new h(k5, i2, abstractC2584oX);
        Iterator it = set.iterator();
        while (it.hasNext()) {
            m mVar = (m) it.next();
            if (!mVar.c.j()) {
                if (mVar.b.j()) {
                    boolean z = i;
                    if (!z && this.e.d(mVar.c.l) >= 256) {
                        throw new AssertionError();
                    }
                    AbstractC1076Vw abstractC1076Vw = mVar.b.b.c;
                    if (abstractC1076Vw.r2()) {
                        hVar.a(new g(mVar.c.l, mVar.a, abstractC1076Vw));
                    } else if (!z && !abstractC1076Vw.l1()) {
                        throw new AssertionError();
                    }
                }
                if (mVar.c.l != mVar.b.l) {
                    C3200vB c3200vBM = this.e.a.M();
                    c3200vBM.getClass();
                    if (c3200vBM.a(EnumC3471y2.y) && mVar.b.b.H() && mVar.a.K() && this.e.d(mVar.c.l) < 256) {
                        hVar.a(new g(mVar.c.l, mVar.a, mVar.b.b.c));
                    } else {
                        hVar.a(new g(mVar.c.l, mVar.b.l, mVar.a));
                    }
                }
            }
        }
        hVar.a();
        this.h = Math.max(this.h, hVar.c);
    }

    public final boolean a(AbstractC1076Vw abstractC1076Vw) {
        int i2 = abstractC1076Vw.e - 1;
        return this.b.containsKey(Integer.valueOf(i2)) || this.a.containsKey(Integer.valueOf(i2)) || this.c.containsKey(Integer.valueOf(i2));
    }
}
