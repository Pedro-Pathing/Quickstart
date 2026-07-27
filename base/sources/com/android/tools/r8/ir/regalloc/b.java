package com.android.tools.r8.ir.regalloc;

import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.AbstractC2584oX;
import com.android.tools.r8.internal.C0582Fd;
import com.android.tools.r8.internal.C0767Le;
import com.android.tools.r8.internal.C0831Nh;
import com.android.tools.r8.internal.C0848Nt;
import com.android.tools.r8.internal.C0874Ot;
import com.android.tools.r8.internal.C1105Wz;
import com.android.tools.r8.internal.C1163Yz;
import com.android.tools.r8.internal.C1190Zz;
import com.android.tools.r8.internal.C1196a2;
import com.android.tools.r8.internal.C1656eq;
import com.android.tools.r8.internal.C1762fq;
import com.android.tools.r8.internal.C2331lg0;
import com.android.tools.r8.internal.C2633p2;
import com.android.tools.r8.internal.C2806qz;
import com.android.tools.r8.internal.C3126uP;
import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.internal.C3177uz;
import com.android.tools.r8.internal.C3200vB;
import com.android.tools.r8.internal.C3214vP;
import com.android.tools.r8.internal.C3323wV;
import com.android.tools.r8.internal.C3465xz;
import com.android.tools.r8.internal.C3607zS;
import com.android.tools.r8.internal.EnumC3471y2;
import com.android.tools.r8.internal.Fk0;
import com.android.tools.r8.internal.H5;
import com.android.tools.r8.internal.InterfaceC2356lz;
import com.android.tools.r8.internal.J5;
import com.android.tools.r8.internal.JL$$ExternalSyntheticLambda0;
import com.android.tools.r8.internal.K5;
import com.android.tools.r8.internal.Ot$$ExternalSyntheticLambda11;
import com.android.tools.r8.internal.Rd0;
import com.android.tools.r8.internal.SW;
import com.android.tools.r8.internal.Sd0;
import com.android.tools.r8.internal.WS;
import com.android.tools.r8.internal.Zf0;
import com.android.tools.r8.internal.bn0;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.IdentityHashMap;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.SortedSet;
import java.util.TreeSet;
import java.util.function.BiPredicate;
import java.util.function.IntConsumer;
import java.util.function.Predicate;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public class b implements f {
    public static final /* synthetic */ boolean r = true;
    public final C0421y a;
    public final C0874Ot b;
    public final int c;
    public IdentityHashMap d;
    public C3161ul0 e;
    public C3161ul0 f;
    public int g = 1;
    public TreeSet h = new TreeSet();
    public int i = -1;
    public ArrayList j = new ArrayList();
    public LinkedList k = new LinkedList();
    protected List<c> l = new LinkedList();
    protected PriorityQueue<c> m = new PriorityQueue<>();
    public final C1105Wz n = new C1105Wz(16);
    public final ArrayList o = new ArrayList();
    public int p = Integer.MIN_VALUE;
    public int[] q = null;

    public b(C0421y<?> c0421y, C0874Ot c0874Ot) {
        this.a = c0421y;
        this.b = c0874Ot;
        int iO = 0;
        for (AbstractC1076Vw abstractC1076Vw : c0874Ot.j().k()) {
            if (abstractC1076Vw.l1()) {
                iO += abstractC1076Vw.d().o.O();
            }
        }
        this.c = iO;
    }

    public final int a(c cVar, int i, j jVar, boolean z, i iVar) {
        int i2 = -1;
        int i3 = -1;
        for (int i4 = 0; i4 <= i; i4++) {
            if (!jVar.b(i4) && ((!z || !jVar.b(i4 + 1)) && jVar.a(i4, iVar))) {
                int iA = jVar.a(i4);
                if (z) {
                    if (i4 == this.c - 1) {
                        continue;
                    } else {
                        if (i4 >= i) {
                            break;
                        }
                        iA = Math.min(iA, jVar.a(i4 + 1));
                    }
                }
                if ((cVar.j.isEmpty() || iA != cVar.b()) && iA > i2) {
                    i3 = i4;
                    if (iA == Integer.MAX_VALUE) {
                        break;
                    }
                    i2 = iA;
                }
            }
        }
        return i3;
    }

    /* JADX WARN: Removed duplicated region for block: B:156:0x02ef  */
    @Override // com.android.tools.r8.ir.regalloc.f
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public final void b() {
        /*
            Method dump skipped, instruction units count: 976
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.android.tools.r8.ir.regalloc.b.b():void");
    }

    public final boolean c(c cVar) {
        C3200vB c3200vBM = this.a.M();
        c3200vBM.getClass();
        if (!c3200vBM.a(EnumC3471y2.w) || cVar.k() == 1 || cVar.b.l() || cVar.e != cVar) {
            return false;
        }
        AbstractC1076Vw abstractC1076Vw = cVar.b.c;
        if (abstractC1076Vw.m1() && abstractC1076Vw.u().K2() == WS.f) {
            return (abstractC1076Vw instanceof C1196a2) || (abstractC1076Vw instanceof C2331lg0);
        }
        if (abstractC1076Vw.f2() && abstractC1076Vw.l0().K2() == WS.f) {
            return (abstractC1076Vw instanceof C3323wV) || (abstractC1076Vw instanceof bn0) || (abstractC1076Vw instanceof C2633p2);
        }
        return false;
    }

    public final boolean d(c cVar) {
        C3200vB c3200vBM = this.a.M();
        c3200vBM.getClass();
        if (!c3200vBM.a(EnumC3471y2.w)) {
            C3200vB c3200vBM2 = this.a.M();
            c3200vBM2.getClass();
            if (!c3200vBM2.a(EnumC3471y2.y)) {
                return false;
            }
        }
        if (cVar.k() == 2 || cVar.b.l() || cVar.e != cVar) {
            return false;
        }
        AbstractC1076Vw abstractC1076Vw = cVar.b.c;
        abstractC1076Vw.getClass();
        if (abstractC1076Vw instanceof C0582Fd) {
            return ((C3161ul0) abstractC1076Vw.c.get(0)).Y().b();
        }
        if (!abstractC1076Vw.p2()) {
            return false;
        }
        C3607zS c3607zSX0 = abstractC1076Vw.x0();
        return c3607zSX0.i == WS.f && c3607zSX0.j == WS.e;
    }

    @Override // com.android.tools.r8.ir.regalloc.f
    public final int e() {
        int i = this.i + 1;
        int[] iArr = this.q;
        return iArr != null ? i - iArr[iArr.length - 1] : i;
    }

    public final void f() {
        this.b.d.forEach(new JL$$ExternalSyntheticLambda0());
    }

    public final void g(c cVar) {
        c cVar2;
        int i = cVar.l;
        boolean zB = cVar.b.Y().b();
        if (!r && !c(i, zB)) {
            throw new AssertionError();
        }
        this.h.remove(Integer.valueOf(i));
        if (zB) {
            this.h.remove(Integer.valueOf(i + 1));
        }
        if (!cVar.h() || cVar == (cVar2 = cVar.e) || cVar2.l == cVar.l) {
            return;
        }
        g(cVar2);
    }

    public final int h() {
        if (r || i()) {
            return this.c;
        }
        throw new AssertionError();
    }

    public final boolean i() {
        return !this.o.isEmpty();
    }

    public final void j() {
        int i;
        Iterator it = this.j.iterator();
        while (true) {
            int i2 = 0;
            if (!it.hasNext()) {
                n nVar = new n(this.a, this.b, this);
                for (c cVar : this.j) {
                    if (cVar.g()) {
                        PriorityQueue priorityQueue = new PriorityQueue();
                        priorityQueue.addAll(cVar.f);
                        c cVar2 = cVar;
                        for (c cVar3 = (c) priorityQueue.poll(); cVar3 != null; cVar3 = (c) priorityQueue.poll()) {
                            int iE = cVar3.e();
                            if (iE % 2 != 1) {
                                iE--;
                            }
                            boolean z = n.i;
                            if (!z && iE % 2 != 1) {
                                throw new AssertionError();
                            }
                            if (!z && cVar3.e != cVar2.e) {
                                throw new AssertionError();
                            }
                            if (((H5) nVar.g.get(Integer.valueOf(iE + 1))) == null) {
                                nVar.a(iE, cVar3, cVar2);
                            }
                            cVar2 = cVar3;
                        }
                    }
                }
                for (H5 h5 : this.b.d) {
                    for (H5 h52 : h5.t()) {
                        int i3 = h5.h().e;
                        boolean zC = h5.c(h52);
                        if (zC) {
                            Iterator<AbstractC1076Vw> it2 = h5.k().iterator();
                            while (true) {
                                if (!it2.hasNext()) {
                                    break;
                                }
                                AbstractC1076Vw next = it2.next();
                                if (next.i()) {
                                    i3 = next.e;
                                    break;
                                }
                            }
                        }
                        int i4 = ((AbstractC1076Vw) h52.f.get(i2)).e;
                        Iterator it3 = ((C0848Nt) this.d.get(h52)).a.iterator();
                        while (it3.hasNext()) {
                            c cVar4 = ((C3161ul0) it3.next()).j;
                            c cVarC = cVar4.c(i3);
                            c cVarC2 = cVar4.c(i4);
                            if (cVarC != cVarC2) {
                                if (!h5.h().L1() || zC) {
                                    int i5 = i4 - 1;
                                    if (!n.i && cVarC2.e != cVarC.e) {
                                        throw new AssertionError();
                                    }
                                    nVar.a(i5, cVarC2, cVarC);
                                } else {
                                    int i6 = i3 - 1;
                                    if (!n.i && cVarC2.e != cVarC.e) {
                                        throw new AssertionError();
                                    }
                                    nVar.b(i6, cVarC2, cVarC);
                                }
                            }
                        }
                        int iIndexOf = h52.s().indexOf(h5);
                        for (SW sw : h52.q()) {
                            c cVarC3 = sw.j.c(i4);
                            c cVarC4 = ((C3161ul0) sw.s.get(iIndexOf)).j.c(i3);
                            if (cVarC4 != cVarC3 && !cVarC3.h()) {
                                if (!r && h5.t().size() != 1) {
                                    throw new AssertionError();
                                }
                                nVar.c(i3 - 1, cVarC3, cVarC4);
                            }
                        }
                        i2 = 0;
                    }
                    i2 = 0;
                }
                int i7 = this.i;
                int i8 = i7 + 1;
                this.p = i8;
                this.i = nVar.a(i8) + i7;
                return;
            }
            c cVar5 = (c) it.next();
            if (!c.s && cVar5.e != cVar5) {
                throw new AssertionError();
            }
            if (cVar5.b.F()) {
                cVar5.r = true;
            } else if (cVar5.b.H()) {
                Iterator it4 = cVar5.b.a0().iterator();
                while (true) {
                    if (it4.hasNext()) {
                        SW sw2 = (SW) it4.next();
                        if (d(sw2.j.l) >= 255) {
                            for (0; i < sw2.c0().size(); i + 1) {
                                i = (((C3161ul0) sw2.s.get(i)) == cVar5.b && cVar5.c(sw2.r.s().get(i).h().e).n) ? 0 : i + 1;
                            }
                        }
                    } else if (cVar5.c() == Integer.MIN_VALUE) {
                        boolean z2 = c.s;
                        if (!z2) {
                            if (!z2 && !cVar5.n) {
                                throw new AssertionError();
                            }
                            for (c cVar6 : cVar5.f) {
                                if (!c.s && !cVar6.n) {
                                    throw new AssertionError();
                                }
                            }
                        }
                        cVar5.r = true;
                    } else {
                        cVar5.r = d(cVar5.c()) < 255;
                    }
                }
            } else {
                continue;
            }
        }
    }

    public final void k() {
        Iterator<H5> it = this.b.d.iterator();
        while (it.hasNext()) {
            K5 k5A = it.next().a(this.b);
            while (k5A.hasNext()) {
                if (a(k5A.next())) {
                    k5A.remove();
                }
            }
        }
    }

    public final boolean l() {
        boolean z;
        boolean z2 = false;
        for (C3161ul0 c3161ul0 = this.e; c3161ul0 != null; c3161ul0 = c3161ul0.h) {
            c cVar = c3161ul0.j;
            if (!r && cVar.p != 65535) {
                throw new AssertionError();
            }
            Iterator it = cVar.f.iterator();
            boolean z3 = true;
            while (true) {
                if (!it.hasNext()) {
                    z = true;
                    break;
                }
                int i = ((c) it.next()).p;
                if (i < 65535) {
                    if (i < e() - 1) {
                        z = false;
                        z3 = false;
                        break;
                    }
                    z3 = false;
                }
            }
            if (z && !z3) {
                for (c cVar2 : cVar.f) {
                    cVar2.l = Integer.MIN_VALUE;
                    cVar2.m = null;
                    cVar2.f(cVar.l);
                    cVar2.a(false);
                }
                z2 = true;
            }
        }
        return z2;
    }

    public final String toString() {
        StringBuilder sb = new StringBuilder("Live ranges:\n");
        for (c cVar : this.j) {
            sb.append(cVar.f());
            sb.append(" ");
            sb.append(cVar);
        }
        sb.append("\nLive range ascii art: \n");
        for (c cVar2 : this.j) {
            C3161ul0 c3161ul0F = cVar2.f();
            if (cVar2.d() == Integer.MIN_VALUE) {
                Zf0.b(20, c3161ul0F + " (no reg): ", sb);
            } else {
                Zf0.b(20, c3161ul0F + " r" + cVar2.d() + ": ", sb);
            }
            sb.append("|");
            sb.append(cVar2.m());
            sb.append("\n");
        }
        return sb.toString();
    }

    public final boolean f(c cVar) {
        if (!r && cVar.l == Integer.MIN_VALUE) {
            throw new AssertionError();
        }
        int i = cVar.l;
        return (this.h.contains(Integer.valueOf(i)) || (cVar.b.Y().b() && this.h.contains(Integer.valueOf(i + 1)))) ? false : true;
    }

    public final boolean e(c cVar) {
        if (!i()) {
            return false;
        }
        if (this.o.size() > 500) {
            return true;
        }
        Iterator it = this.o.iterator();
        while (it.hasNext()) {
            if (cVar.a((c) it.next())) {
                return true;
            }
        }
        return false;
    }

    public final boolean g() {
        if (e() == 0) {
            return false;
        }
        HashSet hashSet = new HashSet();
        for (c cVar : this.j) {
            if (!cVar.j()) {
                hashSet.add(Integer.valueOf(c(cVar.l)));
                if (cVar.b.Y().b()) {
                    hashSet.add(Integer.valueOf(c(cVar.l + 1)));
                }
            }
            for (c cVar2 : cVar.f) {
                if (!cVar2.j()) {
                    hashSet.add(Integer.valueOf(c(cVar2.l)));
                    if (cVar2.b.Y().b()) {
                        hashSet.add(Integer.valueOf(c(cVar2.l + 1)));
                    }
                }
            }
        }
        for (int i = this.p; i < this.i + 1; i++) {
            hashSet.add(Integer.valueOf(c(i)));
        }
        int[] iArr = new int[e()];
        int i2 = 0;
        for (int i3 = 0; i3 < e(); i3++) {
            if (!hashSet.contains(Integer.valueOf(i3))) {
                i2++;
            }
            iArr[i3] = i2;
        }
        this.q = iArr;
        return i2 > 0;
    }

    @Override // com.android.tools.r8.ir.regalloc.f
    public final C3200vB c() {
        return this.a.M();
    }

    public final int c(int i) {
        int iD = d(i);
        int[] iArr = this.q;
        return iArr != null ? iD - iArr[iD] : iD;
    }

    public final boolean a(c cVar, int i, k kVar, boolean z, int i2) {
        if (i2 == Integer.MIN_VALUE || i2 + (z ? 1 : 0) > i || kVar.b(i2) || (z && kVar.b(i2 + 1))) {
            return false;
        }
        int iA = kVar.a(i2);
        if (z) {
            iA = Math.min(iA, kVar.a(i2 + 1));
        }
        if (iA >= cVar.a()) {
            if (c(cVar) && c(cVar, i2)) {
                return false;
            }
            if (b(cVar) && b(cVar, i2)) {
                return false;
            }
            a(cVar, i2);
            g(cVar);
            this.k.add(cVar);
            return true;
        }
        return false;
    }

    @Override // com.android.tools.r8.ir.regalloc.f
    public final C0421y d() {
        return this.a;
    }

    public final boolean c(c cVar, int i) {
        int i2;
        boolean z = r;
        if (!z && !c(cVar)) {
            throw new AssertionError();
        }
        C3161ul0 c3161ul0 = (C3161ul0) cVar.b.c.A().c.get(0);
        C3161ul0 c3161ul0P2 = cVar.b.c.A().P2();
        int i3 = c3161ul0.j.c(cVar.e()).l;
        int i4 = c3161ul0P2.j.c(cVar.e()).l;
        if (z || !(i3 == Integer.MIN_VALUE || i4 == Integer.MIN_VALUE)) {
            return i == i3 + 1 || (i2 = i + 1) == i3 || i == i4 + 1 || i2 == i4;
        }
        throw new AssertionError();
    }

    public final int d(int i) {
        boolean z = r;
        if (!z && i == Integer.MIN_VALUE) {
            throw new AssertionError();
        }
        if (!z && i < 0) {
            throw new AssertionError();
        }
        int i2 = this.c;
        return i < i2 ? this.i - ((i2 - i) - 1) : i - i2;
    }

    public final boolean d(c cVar, int i) {
        boolean z = r;
        if (!z && !d(cVar)) {
            throw new AssertionError();
        }
        AbstractC1076Vw abstractC1076Vw = cVar.b.c;
        abstractC1076Vw.getClass();
        if (abstractC1076Vw instanceof C0582Fd) {
            C3161ul0 c3161ul0 = (C3161ul0) cVar.b.c.C().c.get(0);
            C3161ul0 c3161ul0P2 = cVar.b.c.C().P2();
            int i2 = c3161ul0.j.c(cVar.e()).l;
            int i3 = c3161ul0P2.j.c(cVar.e()).l;
            if (!z && i2 == Integer.MIN_VALUE) {
                throw new AssertionError();
            }
            if (z || i3 != Integer.MIN_VALUE) {
                return i == i2 || i == i2 + 1 || i == i3 || i == i3 + 1;
            }
            throw new AssertionError();
        }
        if (z || cVar.b.c.p2()) {
            return i == ((C3161ul0) cVar.b.c.x0().c.get(0)).j.c(cVar.e()).l;
        }
        throw new AssertionError();
    }

    public final boolean c(int i, boolean z) {
        if (this.h.contains(Integer.valueOf(i)) || (i() && i == h())) {
            if (!z) {
                return true;
            }
            int i2 = i + 1;
            if (this.h.contains(Integer.valueOf(i2))) {
                return true;
            }
            if (i() && i2 == h()) {
                return true;
            }
        }
        return false;
    }

    @Override // com.android.tools.r8.ir.regalloc.f
    public final boolean a(H5 h5, H5 h52) {
        return Objects.equals(h5.a, h52.a);
    }

    @Override // com.android.tools.r8.ir.regalloc.f
    public final A5 a() {
        return this.b.i();
    }

    /* JADX WARN: Code restructure failed: missing block: B:209:0x0471, code lost:
    
        r3 = r3;
        r0 = r23;
     */
    /* JADX WARN: Removed duplicated region for block: B:135:0x0332  */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public static void a(com.android.tools.r8.internal.C0874Ot r20, com.android.tools.r8.internal.AbstractC0695Iu r21, java.util.ArrayList r22, com.android.tools.r8.ir.regalloc.f r23, java.util.IdentityHashMap r24) {
        /*
            Method dump skipped, instruction units count: 1145
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.android.tools.r8.ir.regalloc.b.a(com.android.tools.r8.internal.Ot, com.android.tools.r8.internal.Iu, java.util.ArrayList, com.android.tools.r8.ir.regalloc.f, java.util.IdentityHashMap):void");
    }

    public static boolean b(AbstractC1076Vw abstractC1076Vw) {
        abstractC1076Vw.getClass();
        return ((abstractC1076Vw instanceof C3214vP) || a(abstractC1076Vw)) ? false : true;
    }

    @Override // com.android.tools.r8.ir.regalloc.f
    public final int b(C3161ul0 c3161ul0, int i) {
        if (c3161ul0.F()) {
            return c(c3161ul0.j.l);
        }
        return a(c3161ul0, i);
    }

    public final boolean b(c cVar) {
        C3200vB c3200vBM = this.a.M();
        c3200vBM.getClass();
        if (c3200vBM.b(EnumC3471y2.D) || cVar.k() == 1 || cVar.b.l() || cVar.e != cVar) {
            return false;
        }
        AbstractC1076Vw abstractC1076Vw = cVar.b.c;
        return abstractC1076Vw.o1() && abstractC1076Vw.w().I2().b();
    }

    /* JADX WARN: Code restructure failed: missing block: B:309:0x0564, code lost:
    
        throw new java.lang.AssertionError();
     */
    /* JADX WARN: Code restructure failed: missing block: B:535:0x09c8, code lost:
    
        if (r13 == false) goto L537;
     */
    /* JADX WARN: Code restructure failed: missing block: B:536:0x09ca, code lost:
    
        j();
     */
    /* JADX WARN: Code restructure failed: missing block: B:537:0x09cd, code lost:
    
        r0 = com.android.tools.r8.AbstractC0029c.b(r19);
     */
    /* JADX WARN: Code restructure failed: missing block: B:538:0x09d3, code lost:
    
        if (r0 == 0) goto L566;
     */
    /* JADX WARN: Code restructure failed: missing block: B:539:0x09d5, code lost:
    
        if (r0 == 1) goto L553;
     */
    /* JADX WARN: Code restructure failed: missing block: B:541:0x09d8, code lost:
    
        if (r0 == 2) goto L543;
     */
    /* JADX WARN: Code restructure failed: missing block: B:544:0x09de, code lost:
    
        if (com.android.tools.r8.ir.regalloc.b.r != false) goto L549;
     */
    /* JADX WARN: Code restructure failed: missing block: B:545:0x09e0, code lost:
    
        if (r13 == false) goto L547;
     */
    /* JADX WARN: Code restructure failed: missing block: B:548:0x09e8, code lost:
    
        throw new java.lang.AssertionError();
     */
    /* JADX WARN: Code restructure failed: missing block: B:550:0x09ed, code lost:
    
        if (l() == false) goto L552;
     */
    /* JADX WARN: Code restructure failed: missing block: B:551:0x09ef, code lost:
    
        k();
        j();
     */
    /* JADX WARN: Code restructure failed: missing block: B:552:0x09f5, code lost:
    
        g();
     */
    /* JADX WARN: Code restructure failed: missing block: B:554:0x09fb, code lost:
    
        if (com.android.tools.r8.ir.regalloc.b.r != false) goto L559;
     */
    /* JADX WARN: Code restructure failed: missing block: B:555:0x09fd, code lost:
    
        if (r13 == false) goto L557;
     */
    /* JADX WARN: Code restructure failed: missing block: B:558:0x0a05, code lost:
    
        throw new java.lang.AssertionError();
     */
    /* JADX WARN: Code restructure failed: missing block: B:560:0x0a0a, code lost:
    
        if (l() == false) goto L562;
     */
    /* JADX WARN: Code restructure failed: missing block: B:561:0x0a0c, code lost:
    
        k();
        j();
     */
    /* JADX WARN: Code restructure failed: missing block: B:562:0x0a12, code lost:
    
        g();
     */
    /* JADX WARN: Code restructure failed: missing block: B:563:0x0a1a, code lost:
    
        if ((e() - 1) > r15) goto L565;
     */
    /* JADX WARN: Code restructure failed: missing block: B:564:0x0a1c, code lost:
    
        r18.a.M().u1.getClass();
     */
    /* JADX WARN: Code restructure failed: missing block: B:565:0x0a28, code lost:
    
        r18.q = null;
        r0 = b(3, true);
     */
    /* JADX WARN: Code restructure failed: missing block: B:566:0x0a32, code lost:
    
        if (r13 == false) goto L577;
     */
    /* JADX WARN: Code restructure failed: missing block: B:568:0x0a39, code lost:
    
        if ((e() - 1) > 15) goto L577;
     */
    /* JADX WARN: Code restructure failed: missing block: B:569:0x0a3b, code lost:
    
        r18.a.M().u1.getClass();
     */
    /* JADX WARN: Code restructure failed: missing block: B:570:0x0a48, code lost:
    
        if (com.android.tools.r8.ir.regalloc.b.r != false) goto L576;
     */
    /* JADX WARN: Code restructure failed: missing block: B:572:0x0a4e, code lost:
    
        if (g() != false) goto L574;
     */
    /* JADX WARN: Code restructure failed: missing block: B:575:0x0a56, code lost:
    
        throw new java.lang.AssertionError();
     */
    /* JADX WARN: Code restructure failed: missing block: B:576:0x0a57, code lost:
    
        r0 = r19;
     */
    /* JADX WARN: Code restructure failed: missing block: B:577:0x0a59, code lost:
    
        r0 = b(2, true);
     */
    /* JADX WARN: Code restructure failed: missing block: B:578:0x0a60, code lost:
    
        r2 = com.android.tools.r8.ir.regalloc.b.r;
     */
    /* JADX WARN: Code restructure failed: missing block: B:579:0x0a62, code lost:
    
        if (r2 != false) goto L586;
     */
    /* JADX WARN: Code restructure failed: missing block: B:580:0x0a64, code lost:
    
        if (r0 != 1) goto L586;
     */
    /* JADX WARN: Code restructure failed: missing block: B:582:0x0a6b, code lost:
    
        if ((e() - 1) > 15) goto L584;
     */
    /* JADX WARN: Code restructure failed: missing block: B:585:0x0a73, code lost:
    
        throw new java.lang.AssertionError();
     */
    /* JADX WARN: Code restructure failed: missing block: B:586:0x0a74, code lost:
    
        if (r2 != false) goto L594;
     */
    /* JADX WARN: Code restructure failed: missing block: B:588:0x0a77, code lost:
    
        if (r0 != 2) goto L594;
     */
    /* JADX WARN: Code restructure failed: missing block: B:590:0x0a7e, code lost:
    
        if ((e() - 1) > r15) goto L592;
     */
    /* JADX WARN: Code restructure failed: missing block: B:593:0x0a86, code lost:
    
        throw new java.lang.AssertionError();
     */
    /* JADX WARN: Code restructure failed: missing block: B:594:0x0a87, code lost:
    
        return r0;
     */
    /* JADX WARN: Removed duplicated region for block: B:292:0x0523  */
    /* JADX WARN: Removed duplicated region for block: B:296:0x0543  */
    /* JADX WARN: Removed duplicated region for block: B:312:0x056d A[ADDED_TO_REGION] */
    /* JADX WARN: Removed duplicated region for block: B:316:0x0574 A[ADDED_TO_REGION] */
    /* JADX WARN: Removed duplicated region for block: B:320:0x0584  */
    /* JADX WARN: Removed duplicated region for block: B:326:0x059b  */
    /* JADX WARN: Removed duplicated region for block: B:340:0x05cf  */
    /* JADX WARN: Removed duplicated region for block: B:366:0x0621  */
    /* JADX WARN: Removed duplicated region for block: B:391:0x066c  */
    /* JADX WARN: Removed duplicated region for block: B:397:0x0693  */
    /* JADX WARN: Removed duplicated region for block: B:419:0x073f  */
    /* JADX WARN: Removed duplicated region for block: B:422:0x0750  */
    /* JADX WARN: Removed duplicated region for block: B:479:0x0853  */
    /* JADX WARN: Removed duplicated region for block: B:482:0x0857  */
    /* JADX WARN: Removed duplicated region for block: B:485:0x0869  */
    /* JADX WARN: Removed duplicated region for block: B:488:0x0879  */
    /* JADX WARN: Removed duplicated region for block: B:638:0x0688 A[SYNTHETIC] */
    /* JADX WARN: Removed duplicated region for block: B:641:0x0730 A[SYNTHETIC] */
    /* JADX WARN: Removed duplicated region for block: B:644:0x096d A[SYNTHETIC] */
    /* JADX WARN: Removed duplicated region for block: B:648:0x08d4 A[SYNTHETIC] */
    /* JADX WARN: Removed duplicated region for block: B:650:0x0889 A[SYNTHETIC] */
    /* JADX WARN: Removed duplicated region for block: B:655:0x0753 A[SYNTHETIC] */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public final int b(int r19, boolean r20) {
        /*
            Method dump skipped, instruction units count: 2696
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.android.tools.r8.ir.regalloc.b.b(int, boolean):int");
    }

    public final void a(c cVar, C1190Zz c1190Zz) {
        c cVar2;
        int i = cVar.l;
        if (!r && i == Integer.MIN_VALUE) {
            throw new AssertionError();
        }
        for (int i2 = 0; i2 < cVar.k(); i2++) {
            int i3 = i + i2;
            if (this.h.remove(Integer.valueOf(i3))) {
                c1190Zz.add(i3);
            }
        }
        if (!cVar.h() || cVar == (cVar2 = cVar.e) || cVar2.l == i) {
            return;
        }
        a(cVar2, c1190Zz);
    }

    public static /* synthetic */ boolean a(Set set, AbstractC1076Vw abstractC1076Vw, a aVar) {
        return (set.contains(aVar.b) && a(abstractC1076Vw, aVar)) ? false : true;
    }

    public static boolean a(AbstractC1076Vw abstractC1076Vw, a aVar) {
        int i = aVar.e;
        int i2 = aVar.f;
        C3161ul0 c3161ul0 = aVar.b;
        int i3 = abstractC1076Vw.e;
        if (!r && i >= i3) {
            throw new AssertionError();
        }
        if (i3 >= i2) {
            if (i3 == i2) {
                Iterator it = abstractC1076Vw.c.iterator();
                while (true) {
                    if (it.hasNext()) {
                        C3161ul0 c3161ul02 = (C3161ul0) it.next();
                        if (c3161ul0 == c3161ul02 || (c3161ul0.l() && (c3161ul02 instanceof C1656eq) && ((C1656eq) c3161ul02).r == c3161ul0)) {
                            break;
                        }
                    } else {
                        for (C3161ul0 c3161ul03 : abstractC1076Vw.T0()) {
                            if (c3161ul0 != c3161ul03 && (!c3161ul0.l() || !(c3161ul03 instanceof C1656eq) || ((C1656eq) c3161ul03).r != c3161ul0)) {
                            }
                        }
                    }
                }
            }
            return false;
        }
        return true;
    }

    public static C0831Nh a(C3465xz c3465xz, C3465xz c3465xz2, AbstractC2584oX abstractC2584oX) {
        C0831Nh c0831Nh;
        if (!r && abstractC2584oX.n()) {
            throw new AssertionError();
        }
        if (c3465xz.isEmpty() && c3465xz2.isEmpty()) {
            return null;
        }
        if (!c3465xz.isEmpty() && !c3465xz2.isEmpty()) {
            C1190Zz c1190Zz = new C1190Zz(Math.min(c3465xz.i, c3465xz2.i));
            C2806qz c2806qz = new C2806qz(((C3177uz) c3465xz.c()).b);
            while (c2806qz.hasNext()) {
                InterfaceC2356lz interfaceC2356lz = (InterfaceC2356lz) c2806qz.next();
                if (c3465xz2.get(interfaceC2356lz.a()) == interfaceC2356lz.getValue()) {
                    c1190Zz.add(interfaceC2356lz.a());
                }
            }
            int i = c1190Zz.c;
            if (i == c3465xz.i && i == c3465xz2.i) {
                return null;
            }
            C1163Yz c1163Yz = new C1163Yz(c1190Zz);
            while (c1163Yz.hasNext()) {
                int iQ = c1163Yz.q();
                c3465xz.remove(iQ);
                c3465xz2.remove(iQ);
            }
            c0831Nh = new C0831Nh(c3465xz, c3465xz2);
        } else {
            c0831Nh = new C0831Nh(c3465xz, c3465xz2);
        }
        c0831Nh.b(abstractC2584oX);
        return c0831Nh;
    }

    @Override // com.android.tools.r8.ir.regalloc.f
    public int a(C3161ul0 c3161ul0, int i) {
        if (c3161ul0.L()) {
            return c(c3161ul0.d().c0());
        }
        c cVarQ = c3161ul0.q();
        if (cVarQ != null) {
            if (cVarQ.g()) {
                cVarQ = cVarQ.c(i);
            }
            return c(cVarQ.l);
        }
        throw new C0767Le("Unexpected attempt to get register for a value without a register in method `" + this.b.i().v() + "`.", this.b.i().getOrigin());
    }

    public static boolean a(AbstractC1076Vw abstractC1076Vw) {
        C3161ul0 c3161ul0D = abstractC1076Vw.d();
        if (c3161ul0D == null || !(c3161ul0D instanceof C1762fq)) {
            return false;
        }
        boolean z = r;
        if (!z && abstractC1076Vw.e != -1) {
            throw new AssertionError();
        }
        if (!z && !abstractC1076Vw.i2() && !abstractC1076Vw.A1()) {
            throw new AssertionError();
        }
        if (z || !abstractC1076Vw.E1()) {
            return true;
        }
        throw new AssertionError();
    }

    public static void a(C3161ul0 c3161ul0, H5 h5, int i, ArrayList arrayList, C0874Ot c0874Ot) {
        int i2 = ((AbstractC1076Vw) h5.f.get(0)).e;
        int size = ((h5.k().size() * 2) + i2) - 2;
        int i3 = c3161ul0.l() ? i2 : c3161ul0.c.e;
        if (c3161ul0.j == null) {
            C3161ul0 c3161ul02 = c3161ul0;
            while (true) {
                C3161ul0 c3161ul03 = c3161ul02.i;
                if (c3161ul03 == null) {
                    break;
                } else {
                    c3161ul02 = c3161ul03;
                }
            }
            c cVar = new c(c3161ul02);
            while (true) {
                arrayList.add(cVar);
                c3161ul02 = c3161ul02.h;
                if (c3161ul02 == null) {
                    break;
                }
                c cVar2 = new c(c3161ul02);
                cVar.b(cVar2);
                cVar = cVar2;
            }
        }
        c cVar3 = c3161ul0.j;
        if (i2 <= i3 && i3 <= size) {
            if (c3161ul0.l()) {
                i3--;
            }
            cVar3.a(new e(i3, i));
            if (!r) {
                int i4 = cVar3.p;
                if (!c0874Ot.b.b() && i4 != 65535) {
                    throw new AssertionError();
                }
            }
            if (!c0874Ot.b.b() || c3161ul0.l()) {
                return;
            }
            cVar3.a(new d(i3, c3161ul0.c.G2()));
            return;
        }
        cVar3.a(new e(i2 - 1, i));
    }

    public static /* synthetic */ void a(TreeSet treeSet, int i) {
        if (!r && !treeSet.contains(Integer.valueOf(i))) {
            throw new AssertionError();
        }
        treeSet.remove(Integer.valueOf(i));
    }

    public final void a(c cVar, boolean z) {
        int i;
        C3161ul0 c3161ul0;
        cVar.getClass();
        c cVar2 = cVar;
        while (true) {
            c cVar3 = cVar2.d;
            if (cVar3 == null) {
                break;
            } else {
                cVar2 = cVar3;
            }
        }
        C1190Zz c1190Zz = new C1190Zz();
        for (c cVar4 = cVar2; cVar4 != null; cVar4 = cVar4.c) {
            for (c cVar5 : this.l) {
                if (cVar5.c(cVar4) != -1) {
                    a(cVar5, c1190Zz);
                }
            }
        }
        if (z && (c3161ul0 = this.e) != null) {
            for (c cVar6 = c3161ul0.j; cVar6 != null; cVar6 = cVar6.c) {
                boolean z2 = r;
                if (!z2 && cVar6 != cVar6.e) {
                    throw new AssertionError();
                }
                if (!z2) {
                    c cVar7 = cVar2;
                    while (true) {
                        c cVar8 = cVar7.d;
                        if (cVar8 == null) {
                            break;
                        } else {
                            cVar7 = cVar8;
                        }
                    }
                    if (cVar2 != cVar7) {
                        throw new AssertionError();
                    }
                }
                Iterator it = cVar6.f.iterator();
                while (true) {
                    if (it.hasNext()) {
                        c cVar9 = (c) it.next();
                        if (this.m.contains(cVar9)) {
                            for (c cVar10 = cVar2; cVar10 != null; cVar10 = cVar10.c) {
                                if (cVar9.c(cVar10) != -1) {
                                    a(cVar6, c1190Zz);
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }
        if (e(cVar2) && this.h.remove(Integer.valueOf(h()))) {
            c1190Zz.add(h());
        }
        c cVar11 = cVar2;
        while (true) {
            c cVar12 = cVar11.d;
            if (cVar12 == null) {
                break;
            } else {
                cVar11 = cVar12;
            }
        }
        int i2 = cVar11.k;
        if (i2 != -1) {
            if (!c.s) {
                c cVar13 = cVar2;
                while (true) {
                    c cVar14 = cVar13.d;
                    if (cVar14 == null) {
                        break;
                    } else {
                        cVar13 = cVar14;
                    }
                }
                int iK = 0;
                for (c cVar15 = cVar13; cVar15 != null; cVar15 = cVar15.c) {
                    iK += cVar15.k();
                }
                cVar13.k = iK;
                if (i2 != iK) {
                    throw new AssertionError();
                }
            }
            i = cVar11.k;
        } else {
            c cVar16 = cVar2;
            while (true) {
                c cVar17 = cVar16.d;
                if (cVar17 == null) {
                    break;
                } else {
                    cVar16 = cVar17;
                }
            }
            int iK2 = 0;
            for (c cVar18 = cVar16; cVar18 != null; cVar18 = cVar18.c) {
                iK2 += cVar18.k();
            }
            cVar16.k = iK2;
            i = iK2;
        }
        int iA = a(i, false);
        while (cVar2 != null) {
            cVar2.f(iA);
            boolean z3 = r;
            if (!z3) {
                if (!z3 && cVar2.l == Integer.MIN_VALUE) {
                    throw new AssertionError();
                }
                for (C3161ul0 c3161ul02 = this.e; c3161ul02 != null; c3161ul02 = c3161ul02.h) {
                    if (!r && c3161ul02.j.a(cVar2.l, cVar2.b.Y().b()) && c3161ul02.j.a(cVar2)) {
                        throw new AssertionError();
                    }
                }
            }
            C3161ul0 c3161ul03 = cVar2.b;
            if (!c3161ul03.l() && c3161ul03.c.i2()) {
                c3161ul03.c.n0().L2().j.a(cVar2, this.m);
            }
            if (cVar2 != cVar) {
                this.m.remove(cVar2);
                this.l.add(cVar2);
            }
            iA += cVar2.k();
            cVar2 = cVar2.c;
        }
        if (!r && cVar.l == Integer.MIN_VALUE) {
            throw new AssertionError();
        }
        g(cVar);
        this.k.add(cVar);
        this.h.addAll(c1190Zz);
    }

    public final int a(c cVar, C1105Wz c1105Wz) {
        int iA;
        if (cVar.h()) {
            return cVar.e.l;
        }
        TreeSet treeSet = new TreeSet((SortedSet) this.h);
        int i = this.i;
        this.h.removeAll(this.n);
        if (c1105Wz != null) {
            this.h.removeAll(c1105Wz);
        }
        Iterator it = cVar.e.f.iterator();
        while (true) {
            if (!it.hasNext()) {
                iA = -1;
                break;
            }
            iA = ((c) it.next()).l;
            if (iA != Integer.MIN_VALUE) {
                boolean zB = cVar.b.Y().b();
                if (this.h.contains(Integer.valueOf(iA)) && (!zB || (this.h.contains(Integer.valueOf(iA + 1)) && iA != this.c - 1))) {
                    if (a(iA, i, cVar)) {
                        break;
                    }
                }
            }
        }
        if (iA == -1) {
            do {
                iA = a(cVar.k(), !cVar.j.isEmpty() && ((d) cVar.j.first()).c == 15);
            } while (!a(iA, i, cVar));
        }
        this.h = treeSet;
        for (int i2 = i + 1; i2 <= this.i; i2++) {
            this.h.add(Integer.valueOf(i2));
        }
        if (r || c(iA, cVar.b.Y().b())) {
            return iA;
        }
        throw new AssertionError();
    }

    public final boolean a(int i, int i2, c cVar) {
        c next;
        if (i > i2) {
            return true;
        }
        if (i < this.c) {
            c cVar2 = this.e.j;
            while (!cVar2.a(i, cVar.b.Y().b())) {
                cVar2 = cVar2.c;
                if (!r && cVar2 == null) {
                    throw new AssertionError();
                }
            }
            while (!cVar2.a(cVar)) {
                cVar2 = cVar2.c;
                if (cVar2 == null || !cVar2.a(i, cVar.b.Y().b())) {
                }
            }
            this.h.remove(Integer.valueOf(i));
            if (i == cVar2.l && cVar2.b.Y().b()) {
                this.h.remove(Integer.valueOf(i + 1));
            }
            return false;
        }
        Iterator<c> it = this.l.iterator();
        while (true) {
            if (!it.hasNext()) {
                next = null;
                break;
            }
            next = it.next();
            if (next.a(i, cVar.b.Y().b()) && cVar.c(next) != -1) {
                break;
            }
        }
        if (next != null) {
            this.h.remove(Integer.valueOf(i));
            if (i == next.l && next.b.Y().b()) {
                this.h.remove(Integer.valueOf(i + 1));
            }
            return false;
        }
        if (!i() || ((i != h() && (!cVar.b.Y().b() || i + 1 != h())) || !e(cVar))) {
            return true;
        }
        this.h.remove(Integer.valueOf(i));
        return false;
    }

    public final void a(c cVar, int i) {
        int i2;
        if (!r && (cVar.k() + i) - 1 > this.i) {
            throw new AssertionError();
        }
        cVar.f(i);
        C3161ul0 c3161ul0 = cVar.b;
        Iterator it = c3161ul0.a0().iterator();
        while (true) {
            i2 = 0;
            if (!it.hasNext()) {
                break;
            }
            SW sw = (SW) it.next();
            c cVar2 = sw.j;
            if (cVar2.m == null) {
                cVar2.a(cVar, this.m);
                while (i2 < sw.c0().size()) {
                    c cVarC = ((C3161ul0) sw.s.get(i2)).j.c(sw.r.s().get(i2).h().e);
                    if (cVarC.m == null) {
                        cVarC.a(cVar, this.m);
                    }
                    i2++;
                }
            }
        }
        if (c3161ul0.l() && cVar.e == cVar) {
            SW swP = c3161ul0.p();
            H5 h5 = swP.r;
            while (i2 < swP.c0().size()) {
                ((C3161ul0) swP.s.get(i2)).j.c(h5.s().get(i2).h().e).a(cVar, this.m);
                i2++;
            }
        }
    }

    public final int a(Predicate predicate, BiPredicate biPredicate, int i, c cVar, int i2, boolean z, l lVar, i iVar) {
        if (predicate.test(cVar)) {
            while (biPredicate.test(cVar, Integer.valueOf(i))) {
                lVar.c(i);
                int iA = a(cVar, i2, lVar, z, iVar);
                if (i == iA) {
                    if (r) {
                        return -1;
                    }
                    throw new AssertionError("Unexpected attempt to take blocked register " + iA + " in " + this.b.i().v());
                }
                if (iA == -1) {
                    return iA;
                }
                i = iA;
            }
        }
        return i;
    }

    public final int a(c cVar, int i, boolean z, k kVar, i iVar) {
        int iA = a(cVar, i, kVar, z, iVar);
        if (iA == -1) {
            return iA;
        }
        l lVar = new l(kVar);
        return a(new Predicate() { // from class: com.android.tools.r8.ir.regalloc.b$$ExternalSyntheticLambda6
            @Override // java.util.function.Predicate
            public final boolean test(Object obj) {
                return this.f$0.b((c) obj);
            }
        }, new BiPredicate() { // from class: com.android.tools.r8.ir.regalloc.b$$ExternalSyntheticLambda7
            @Override // java.util.function.BiPredicate
            public final boolean test(Object obj, Object obj2) {
                return this.f$0.b((c) obj, ((Integer) obj2).intValue());
            }
        }, a(new Predicate() { // from class: com.android.tools.r8.ir.regalloc.b$$ExternalSyntheticLambda4
            @Override // java.util.function.Predicate
            public final boolean test(Object obj) {
                return this.f$0.d((c) obj);
            }
        }, new BiPredicate() { // from class: com.android.tools.r8.ir.regalloc.b$$ExternalSyntheticLambda5
            @Override // java.util.function.BiPredicate
            public final boolean test(Object obj, Object obj2) {
                return this.f$0.d((c) obj, ((Integer) obj2).intValue());
            }
        }, a(new Predicate() { // from class: com.android.tools.r8.ir.regalloc.b$$ExternalSyntheticLambda2
            @Override // java.util.function.Predicate
            public final boolean test(Object obj) {
                return this.f$0.c((c) obj);
            }
        }, new BiPredicate() { // from class: com.android.tools.r8.ir.regalloc.b$$ExternalSyntheticLambda3
            @Override // java.util.function.BiPredicate
            public final boolean test(Object obj, Object obj2) {
                return this.f$0.c((c) obj, ((Integer) obj2).intValue());
            }
        }, iA, cVar, i, z, lVar, iVar), cVar, i, z, lVar, iVar), cVar, i, z, lVar, iVar);
    }

    public final void a(c cVar, int i, boolean z) {
        d dVar;
        c cVar2;
        boolean z2 = r;
        int i2 = Integer.MIN_VALUE;
        if (!z2 && cVar.l != Integer.MIN_VALUE) {
            throw new AssertionError();
        }
        if (!z2 && this.h.contains(Integer.valueOf(i)) && (!z || this.h.contains(Integer.valueOf(i + 1)))) {
            throw new AssertionError();
        }
        final C1105Wz c1105Wz = new C1105Wz(z ? 2 : 1);
        c1105Wz.add(i);
        if (z) {
            c1105Wz.add(i + 1);
        }
        if (cVar.h() && cVar != (cVar2 = cVar.e)) {
            cVar2.a(new IntConsumer() { // from class: com.android.tools.r8.ir.regalloc.b$$ExternalSyntheticLambda0
                @Override // java.util.function.IntConsumer
                public final void accept(int i3) {
                    c1105Wz.add(i3);
                }
            });
        }
        ArrayList arrayList = new ArrayList();
        Iterator it = this.k.iterator();
        while (it.hasNext()) {
            c cVar3 = (c) it.next();
            boolean z3 = r;
            if (!z3 && !f(cVar3)) {
                throw new AssertionError();
            }
            if (cVar3.a(i, z)) {
                it.remove();
                int iA = a(cVar3, c1105Wz);
                a(cVar3);
                c cVarG = cVar3.g(cVar.e());
                a(cVarG, iA);
                cVarG.a(true);
                g(cVarG);
                if (!z3 && cVarG.l == i2) {
                    throw new AssertionError();
                }
                if (!z3 && cVar3.l == i2) {
                    throw new AssertionError();
                }
                arrayList.add(cVarG);
                if (cVar3.b.H() && cVar3.e() == cVar3.b.c.e && cVar3.j.size() == 1) {
                    cVar3.a(true);
                }
                if (cVarG.j.size() <= 0) {
                    i2 = Integer.MIN_VALUE;
                } else if (cVarG.i() && !cVarG.h()) {
                    c cVarG2 = cVarG.g(cVarG.b());
                    cVarG2.f(cVar3.l);
                    this.l.add(cVarG2);
                } else if (cVar3.b.H()) {
                    if (!z3 && !cVarG.n) {
                        throw new AssertionError();
                    }
                    if (!z3 && !cVarG.b.H()) {
                        throw new AssertionError();
                    }
                    if (!z3 && cVarG.i() && !cVarG.h()) {
                        throw new AssertionError();
                    }
                    if (cVarG.j.isEmpty()) {
                        i2 = Integer.MIN_VALUE;
                    } else {
                        c cVarG3 = cVarG.g(cVarG.b());
                        this.m.add(cVarG3);
                        boolean z4 = true;
                        while (z4) {
                            int iE = cVarG3.e();
                            Iterator it2 = cVarG3.j.iterator();
                            while (true) {
                                if (!it2.hasNext()) {
                                    z4 = false;
                                    break;
                                }
                                d dVar2 = (d) it2.next();
                                int i3 = dVar2.b;
                                if (i3 - iE > 22) {
                                    c cVarG4 = cVarG3.g(iE + 2);
                                    int i4 = dVar2.b;
                                    if (i4 % 2 != 1) {
                                        i4--;
                                    }
                                    if (i4 > cVarG4.e()) {
                                        a(cVarG4, iA);
                                        cVarG4.a(true);
                                        this.l.add(cVarG4);
                                        cVarG4 = cVarG4.g(dVar2.b);
                                    }
                                    cVarG3 = cVarG4;
                                    this.m.add(cVarG3);
                                    z4 = true;
                                } else {
                                    iE = i3;
                                }
                            }
                        }
                        i2 = Integer.MIN_VALUE;
                    }
                } else if (cVar3.h()) {
                    if (!z3 && !cVarG.n) {
                        throw new AssertionError();
                    }
                    if (!z3 && !cVarG.h()) {
                        throw new AssertionError();
                    }
                    if (cVarG.j.isEmpty()) {
                        i2 = Integer.MIN_VALUE;
                    } else {
                        this.m.add(cVarG.g(((d) cVarG.j.first()).b));
                        i2 = Integer.MIN_VALUE;
                    }
                } else {
                    if (!z3 && !cVarG.n) {
                        throw new AssertionError();
                    }
                    if (!z3 && cVarG.b.H()) {
                        throw new AssertionError();
                    }
                    if (!z3 && cVarG.i() && !cVarG.h()) {
                        throw new AssertionError();
                    }
                    if (cVarG.h() || iA < this.c) {
                        iA = this.g == 2 ? 255 : 65535;
                    }
                    int iK = (cVarG.k() + iA) - 1;
                    Iterator it3 = cVarG.j.iterator();
                    boolean z5 = false;
                    while (true) {
                        if (!it3.hasNext()) {
                            dVar = null;
                            break;
                        }
                        dVar = (d) it3.next();
                        if (iK > dVar.c) {
                            break;
                        } else {
                            z5 = true;
                        }
                    }
                    if (z5) {
                        cVarG.a(false);
                    }
                    if (dVar == null) {
                        i2 = Integer.MIN_VALUE;
                    } else {
                        this.m.add(cVarG.g(dVar.b));
                        i2 = Integer.MIN_VALUE;
                    }
                }
            } else {
                i2 = Integer.MIN_VALUE;
            }
        }
        this.k.addAll(arrayList);
        if (!r && !c(i, z)) {
            throw new AssertionError();
        }
    }

    public static void a(List list, c cVar, int i, k kVar, k kVar2) {
        int i2;
        int iB;
        Iterator it = list.iterator();
        while (it.hasNext()) {
            c cVar2 = (c) it.next();
            if (cVar2.i() && (i2 = cVar2.l) <= i && cVar2.c(cVar) != -1) {
                for (int i3 = 0; i3 < cVar2.k(); i3++) {
                    int i4 = i2 + i3;
                    if (i4 <= i && (iB = cVar2.b(cVar.e())) < kVar2.a(i4)) {
                        kVar2.a(i4, iB, cVar2);
                        if (!r && !kVar.f.get(i4) && kVar.a(i4) > kVar2.a(i4)) {
                            throw new AssertionError();
                        }
                    }
                }
            }
        }
    }

    public static void a(C0421y c0421y, C0874Ot c0874Ot, IdentityHashMap identityHashMap, ArrayList arrayList) {
        Fk0 it = c0874Ot.D().iterator();
        while (it.hasNext()) {
            H5 h5 = (H5) it.next();
            LinkedHashSet<C3161ul0> linkedHashSet = new LinkedHashSet();
            LinkedHashSet linkedHashSet2 = new LinkedHashSet();
            LinkedHashSet<C3161ul0> linkedHashSet3 = new LinkedHashSet();
            Set setA = h5.i().a();
            for (H5 h52 : h5.t()) {
                boolean zContains = setA.contains(h52);
                if (zContains) {
                    linkedHashSet3.addAll(((C0848Nt) identityHashMap.get(h52)).a);
                } else {
                    linkedHashSet.addAll(((C0848Nt) identityHashMap.get(h52)).a);
                }
                if (!r && zContains && !h52.q().isEmpty()) {
                    throw new AssertionError();
                }
                for (SW sw : h52.q()) {
                    linkedHashSet.remove(sw);
                    linkedHashSet2.add((C3161ul0) sw.s.get(h52.s().indexOf(h5)));
                }
            }
            linkedHashSet.addAll(linkedHashSet2);
            LinkedList<AbstractC1076Vw> linkedListK = h5.k();
            for (C3161ul0 c3161ul0 : linkedHashSet) {
                int size = (linkedListK.size() * 2) + ((AbstractC1076Vw) h5.f.get(0)).e;
                if (linkedHashSet2.contains(c3161ul0)) {
                    size--;
                }
                a(c3161ul0, h5, size, arrayList, c0874Ot);
            }
            J5 j5 = new J5(h5, h5.k().size());
            while (j5.b.hasPrevious()) {
                AbstractC1076Vw abstractC1076Vw = (AbstractC1076Vw) j5.b.previous();
                C3161ul0 c3161ul0D = abstractC1076Vw.d();
                if (c3161ul0D != null) {
                    if (c3161ul0D instanceof Sd0) {
                        for (Rd0 rd0 : ((Sd0) c3161ul0D).r) {
                            linkedHashSet.remove(rd0);
                        }
                    } else if (!c3161ul0D.O()) {
                        a(c3161ul0D, h5, abstractC1076Vw.e + 1, arrayList, c0874Ot);
                        if (!r && c0874Ot.b.a() && !abstractC1076Vw.l1()) {
                            throw new AssertionError("Arguments should be the only potentially unused local in CF");
                        }
                    }
                    linkedHashSet.remove(c3161ul0D);
                }
                for (C3161ul0 c3161ul02 : abstractC1076Vw.c) {
                    if (c3161ul02.T()) {
                        if (!r) {
                            if (!(c0874Ot.b.b() || abstractC1076Vw.F2() == 65535)) {
                                throw new AssertionError();
                            }
                        }
                        if (!linkedHashSet.contains(c3161ul02)) {
                            linkedHashSet.add(c3161ul02);
                            a(c3161ul02, h5, abstractC1076Vw.e, arrayList, c0874Ot);
                        }
                        if (c0874Ot.b.b()) {
                            int iF2 = abstractC1076Vw.F2();
                            c cVar = c3161ul02.j;
                            if (!c3161ul02.F() || iF2 != 65535) {
                                cVar.a(new d(abstractC1076Vw.e, iF2));
                            }
                        }
                    }
                }
                if (abstractC1076Vw.i()) {
                    for (C3161ul0 c3161ul03 : linkedHashSet3) {
                        if (c3161ul03.T() && !linkedHashSet.contains(c3161ul03)) {
                            linkedHashSet.add(c3161ul03);
                            int i = abstractC1076Vw.e;
                            if (abstractC1076Vw.v1() && c3161ul03 != abstractC1076Vw.B().h()) {
                                i += 2;
                            }
                            a(c3161ul03, h5, i, arrayList, c0874Ot);
                        }
                    }
                }
                if (c0421y.M().Z0 || c0874Ot.i().a().f(c0421y)) {
                    int i2 = abstractC1076Vw.e;
                    ArrayList<C3161ul0> arrayList2 = new ArrayList(abstractC1076Vw.T0());
                    arrayList2.sort(new Ot$$ExternalSyntheticLambda11());
                    for (C3161ul0 c3161ul04 : arrayList2) {
                        if (!r && !c3161ul04.T()) {
                            throw new AssertionError();
                        }
                        if (!linkedHashSet.contains(c3161ul04)) {
                            linkedHashSet.add(c3161ul04);
                            a(c3161ul04, h5, i2, arrayList, c0874Ot);
                        }
                    }
                }
            }
        }
    }

    public final void b(int i) {
        c cVar;
        final TreeSet treeSet = new TreeSet();
        for (int i2 = 0; i2 <= this.i; i2++) {
            treeSet.add(Integer.valueOf(i2));
        }
        for (c cVar2 : this.k) {
            if (!r && !f(cVar2)) {
                throw new AssertionError();
            }
            cVar2.a(new IntConsumer() { // from class: com.android.tools.r8.ir.regalloc.b$$ExternalSyntheticLambda13
                @Override // java.util.function.IntConsumer
                public final void accept(int i3) {
                    b.a(treeSet, i3);
                }
            });
        }
        if (i == 2 || i == 3) {
            for (c cVar3 : this.k) {
                if (cVar3.h() && cVar3 != (cVar = cVar3.e) && cVar.l != cVar3.l) {
                    cVar.a(new IntConsumer() { // from class: com.android.tools.r8.ir.regalloc.b$$ExternalSyntheticLambda14
                        @Override // java.util.function.IntConsumer
                        public final void accept(int i3) {
                            b.b(treeSet, i3);
                        }
                    });
                }
            }
        }
        if (i()) {
            this.h.remove(Integer.valueOf(h()));
            treeSet.remove(Integer.valueOf(h()));
        }
        if (!r && !this.h.equals(treeSet)) {
            throw new AssertionError();
        }
    }

    public static /* synthetic */ void b(TreeSet treeSet, int i) {
        if (!r && !treeSet.contains(Integer.valueOf(i))) {
            throw new AssertionError();
        }
        treeSet.remove(Integer.valueOf(i));
    }

    public final boolean b(c cVar, int i) {
        boolean z = r;
        if (!z && !b(cVar)) {
            throw new AssertionError();
        }
        int i2 = cVar.b.c.w().K2().j.c(cVar.e()).l;
        if (z || i2 != Integer.MIN_VALUE) {
            return i2 == i;
        }
        throw new AssertionError();
    }

    public void b(c cVar, int i, boolean z) {
        int iB;
        ArrayList arrayList = new ArrayList();
        Iterator<c> it = this.l.iterator();
        while (it.hasNext()) {
            c next = it.next();
            if (next.a(i, z) && next.c(cVar) != -1) {
                if (next.i() && !next.h() && (iB = next.b(cVar.e())) != Integer.MAX_VALUE) {
                    c cVarG = next.g(iB);
                    cVarG.f(next.l);
                    arrayList.add(cVarG);
                }
                if (next.e() > cVar.e()) {
                    next.l = Integer.MIN_VALUE;
                    next.m = null;
                    it.remove();
                    this.m.add(next);
                } else {
                    this.m.add(next.g(cVar.e()));
                }
            }
        }
        this.l.addAll(arrayList);
    }

    @Override // com.android.tools.r8.ir.regalloc.f
    public final void b(H5 h5, H5 h52) {
    }

    public static boolean a(C0874Ot c0874Ot) {
        if (c0874Ot.d.size() > 1) {
            return false;
        }
        J5 j5H = c0874Ot.j().H();
        while (j5H.hasNext() && j5H.m().l1()) {
            j5H.next();
        }
        while (j5H.hasNext() && j5H.m().v1() && j5H.m().B().h().F()) {
            j5H.next();
        }
        if (j5H.hasNext() && j5H.next().S1()) {
            if (j5H.hasNext() && j5H.m().v1()) {
                j5H.next();
            }
            if (j5H.hasNext() && j5H.next().v2()) {
                return true;
            }
        }
        return false;
    }

    /* JADX WARN: Removed duplicated region for block: B:50:0x00fd  */
    /* JADX WARN: Removed duplicated region for block: B:62:0x0128  */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public final void a(com.android.tools.r8.internal.TB r14, com.android.tools.r8.internal.K5 r15) {
        /*
            Method dump skipped, instruction units count: 479
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.android.tools.r8.ir.regalloc.b.a(com.android.tools.r8.internal.TB, com.android.tools.r8.internal.K5):void");
    }

    public static int a(C3126uP c3126uP, C3126uP c3126uP2) {
        return c3126uP.L2().c.e - c3126uP2.L2().c.e;
    }

    public final void a(int i) {
        for (int i2 = this.i + 1; i2 <= i; i2++) {
            this.h.add(Integer.valueOf(i2));
        }
        this.i = i;
    }

    public final int a(int i, boolean z) {
        int iIntValue;
        int i2;
        int iIntValue2;
        int i3 = this.i;
        TreeSet treeSet = this.h;
        if (z) {
            treeSet = new TreeSet(new Comparator() { // from class: com.android.tools.r8.ir.regalloc.b$$ExternalSyntheticLambda1
                @Override // java.util.Comparator
                public final int compare(Object obj, Object obj2) {
                    return this.f$0.a((Integer) obj, (Integer) obj2);
                }
            });
            treeSet.addAll(this.h);
        }
        Iterator it = treeSet.iterator();
        if (it.hasNext()) {
            iIntValue = ((Integer) it.next()).intValue();
        } else {
            iIntValue = this.i + 1;
            this.i = iIntValue;
        }
        int i4 = iIntValue;
        while ((iIntValue - i4) + 1 != i) {
            for (int i5 = 0; i5 < i - 1; i5++) {
                if (it.hasNext()) {
                    iIntValue2 = ((Integer) it.next()).intValue();
                } else {
                    iIntValue2 = this.i + 1;
                    this.i = iIntValue2;
                }
                iIntValue++;
                if (iIntValue2 != iIntValue || iIntValue2 == this.c) {
                    iIntValue = iIntValue2;
                    i4 = iIntValue;
                    break;
                }
            }
        }
        while (true) {
            i3++;
            if (i3 <= this.i) {
                boolean zAdd = this.h.add(Integer.valueOf(i3));
                if (!r && !zAdd) {
                    throw new AssertionError();
                }
            } else {
                if (r || ((i4 < (i2 = this.c) && (i4 + i) - 1 < i2) || (i4 >= i2 && (i + i4) - 1 >= i2))) {
                    return i4;
                }
                throw new AssertionError();
            }
        }
    }

    public final /* synthetic */ int a(Integer num, Integer num2) {
        boolean z = num.intValue() < this.c;
        boolean z2 = num2.intValue() < this.c;
        if (z && !z2) {
            return 1;
        }
        if (z || !z2) {
            return num.intValue() - num2.intValue();
        }
        return -1;
    }

    public final void a(c cVar) {
        c cVar2;
        boolean z = r;
        if (!z && !f(cVar)) {
            throw new AssertionError();
        }
        int i = cVar.l;
        if (!z && (cVar.k() + i) - 1 > this.i) {
            throw new AssertionError();
        }
        this.h.add(Integer.valueOf(i));
        if (cVar.b.Y().b()) {
            this.h.add(Integer.valueOf(i + 1));
        }
        if (!cVar.h() || cVar == (cVar2 = cVar.e) || cVar2.l == cVar.l) {
            return;
        }
        a(cVar2);
    }

    @Override // com.android.tools.r8.ir.regalloc.f
    public final void a(H5 h5, int i, List list) {
    }
}
