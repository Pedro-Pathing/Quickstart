package com.android.tools.r8.ir.regalloc;

import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.C1105Wz;
import com.android.tools.r8.internal.C2843rP;
import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.internal.iX$$ExternalSyntheticLambda2;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;
import java.util.PriorityQueue;
import java.util.TreeSet;
import java.util.function.IntConsumer;
import java.util.function.ToIntFunction;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public class c implements Comparable<c> {
    public static final /* synthetic */ boolean s = true;
    public final C3161ul0 b;
    public c c;
    public c d;
    public final c e;
    public Integer m;
    public final boolean o;
    public final ArrayList f = new ArrayList();
    public final C1105Wz g = new C1105Wz(16);
    public boolean h = false;
    public List i = new ArrayList();
    public final TreeSet j = new TreeSet();
    public int k = -1;
    public int l = Integer.MIN_VALUE;
    public boolean n = false;
    public int p = 65535;
    public int q = Integer.MIN_VALUE;
    public boolean r = false;

    public c(C3161ul0 c3161ul0) {
        boolean z = false;
        this.o = false;
        this.b = c3161ul0;
        Iterator<AbstractC1076Vw> it = c3161ul0.b0().iterator();
        while (true) {
            if (!it.hasNext()) {
                break;
            }
            AbstractC1076Vw next = it.next();
            next.getClass();
            if (next instanceof C2843rP) {
                z = true;
                break;
            }
        }
        this.o = z;
        this.e = this;
        if (!C3161ul0.q && c3161ul0.j != null) {
            throw new AssertionError();
        }
        c3161ul0.j = this;
    }

    public final void a(boolean z) {
        boolean z2 = s;
        if (!z2 && this.l == Integer.MIN_VALUE) {
            throw new AssertionError();
        }
        if (!z2 && z && h() && this.l != this.e.l) {
            throw new AssertionError();
        }
        this.n = z;
    }

    public void b(c cVar) {
        if (!s && this.k != -1) {
            throw new AssertionError();
        }
        this.c = cVar;
        cVar.d = this;
    }

    public final int c() {
        c cVar = this.e;
        int i = cVar.q;
        if (i != Integer.MIN_VALUE) {
            return i;
        }
        boolean z = s;
        if (!z && cVar.e != cVar) {
            throw new AssertionError();
        }
        if (!z && i != Integer.MIN_VALUE) {
            throw new AssertionError();
        }
        if (!cVar.n) {
            cVar.q = cVar.l;
        }
        for (c cVar2 : cVar.f) {
            if (!cVar2.n) {
                cVar.q = Math.max(cVar.q, cVar2.l);
            }
        }
        return cVar.q;
    }

    @Override // java.lang.Comparable
    public final int compareTo(c cVar) {
        int iIntValue;
        c cVar2 = cVar;
        int iE = e() - cVar2.e();
        if (iE != 0) {
            return iE;
        }
        Integer num = this.m;
        if (num != null && cVar2.m != null && (iIntValue = num.intValue() - cVar2.m.intValue()) != 0) {
            return iIntValue;
        }
        Integer num2 = this.m;
        if (num2 != null && cVar2.m == null) {
            return -1;
        }
        if (num2 == null && cVar2.m != null) {
            return 1;
        }
        int iS = this.b.s() - cVar2.b.s();
        if (s || iS != 0) {
            return iS;
        }
        throw new AssertionError();
    }

    public final int d() {
        return this.l;
    }

    public int e() {
        if (s || !this.i.isEmpty()) {
            return ((e) this.i.get(0)).b;
        }
        throw new AssertionError();
    }

    public final C3161ul0 f() {
        return this.b;
    }

    public final c g(int i) {
        e eVar;
        int i2;
        List list;
        int i3 = i % 2;
        int i4 = i3 == 0 ? i : i + 1;
        int iE = e();
        if (iE % 2 != 0) {
            iE++;
        }
        if (i4 == iE) {
            if (!s && this.j.size() != 0 && b() == i) {
                throw new AssertionError();
            }
            this.l = Integer.MIN_VALUE;
            return this;
        }
        if (!s && this.b.c(new iX$$ExternalSyntheticLambda2())) {
            throw new AssertionError();
        }
        if (i3 != 1) {
            i--;
        }
        c cVar = new c(this.e);
        this.e.f.add(cVar);
        this.e.h = false;
        ArrayList arrayList = new ArrayList();
        ArrayList arrayList2 = new ArrayList();
        if (i == a()) {
            List list2 = this.i;
            arrayList2.add(new e(i, i));
            list = list2;
        } else {
            int i5 = 0;
            while (i5 < this.i.size() && (((i2 = (eVar = (e) this.i.get(i5)).b) > i || eVar.c <= i) && i2 <= i)) {
                i5++;
            }
            e eVar2 = (e) this.i.get(i5);
            arrayList.addAll(this.i.subList(0, i5));
            if (eVar2.b < i) {
                arrayList.add(new e(eVar2.b, i));
                arrayList2.add(new e(i, eVar2.c));
            } else {
                arrayList2.add(eVar2);
            }
            List list3 = this.i;
            arrayList2.addAll(list3.subList(i5 + 1, list3.size()));
            list = arrayList;
        }
        cVar.i = arrayList2;
        this.i = list;
        while (!this.j.isEmpty() && ((d) this.j.last()).b >= i) {
            cVar.a((d) this.j.pollLast());
        }
        this.p = 65535;
        Iterator it = this.j.iterator();
        while (it.hasNext()) {
            this.p = Math.min(this.p, ((d) it.next()).c);
        }
        boolean z = s;
        if (!z && this.i.isEmpty()) {
            throw new AssertionError();
        }
        if (z || !cVar.i.isEmpty()) {
            return cVar;
        }
        throw new AssertionError();
    }

    public final boolean h() {
        AbstractC1076Vw abstractC1076Vw = this.e.b.c;
        return abstractC1076Vw != null && abstractC1076Vw.l1();
    }

    public final boolean i() {
        c cVar = this.e;
        return (cVar.d == null && cVar.c == null) ? false : true;
    }

    public final boolean j() {
        if (this.n) {
            c cVar = this.e;
            if (!s && cVar.e != cVar) {
                throw new AssertionError();
            }
            if (cVar.r) {
                return true;
            }
        }
        return false;
    }

    public final int k() {
        return this.b.Y().c();
    }

    public final void l() {
        if (this.h) {
            return;
        }
        this.f.sort(Comparator.comparingInt(new ToIntFunction() { // from class: com.android.tools.r8.ir.regalloc.c$$ExternalSyntheticLambda0
            @Override // java.util.function.ToIntFunction
            public final int applyAsInt(Object obj) {
                return ((c) obj).a();
            }
        }));
        this.g.c = 0;
        Iterator it = this.f.iterator();
        while (it.hasNext()) {
            this.g.add(((c) it.next()).a());
        }
        if (!s) {
            for (int i = 0; i < this.f.size(); i++) {
                boolean z = s;
                if (!z && ((c) this.f.get(i)).a() != this.g.i(i)) {
                    throw new AssertionError();
                }
                if (!z && i != 0 && this.g.i(i - 1) > this.g.i(i)) {
                    throw new AssertionError();
                }
            }
        }
        this.h = true;
    }

    public final String m() {
        StringBuilder sb = new StringBuilder();
        Iterator it = this.i.iterator();
        int i = 0;
        while (true) {
            if (!it.hasNext()) {
                break;
            }
            e eVar = (e) it.next();
            eVar.getClass();
            if (eVar == e.d) {
                sb.append("--- infinite ---...");
                break;
            }
            while (i < eVar.b) {
                sb.append(" ");
                i++;
            }
            while (i < eVar.c) {
                sb.append("-");
                i++;
            }
        }
        return sb.toString();
    }

    public final String toString() {
        StringBuilder sb = new StringBuilder("(cons ");
        sb.append(this.k);
        sb.append("): ");
        Iterator it = this.i.iterator();
        while (it.hasNext()) {
            sb.append((e) it.next());
            sb.append(" ");
        }
        sb.append("\n");
        return sb.toString();
    }

    public void f(int i) {
        int i2;
        if (!s && (i2 = this.l) != Integer.MIN_VALUE && i2 != i) {
            throw new AssertionError();
        }
        this.l = i;
    }

    public final boolean e(int i) {
        for (e eVar : this.i) {
            if (eVar.b > i) {
                return false;
            }
            if (i < eVar.c) {
                return true;
            }
        }
        return false;
    }

    public final int b(int i) {
        Iterator it = this.j.iterator();
        while (it.hasNext()) {
            int i2 = ((d) it.next()).b;
            if (i2 >= i) {
                return i2;
            }
        }
        return Integer.MAX_VALUE;
    }

    public final int b() {
        return ((d) this.j.first()).b;
    }

    public final boolean a(c cVar) {
        c cVar2 = this.e;
        if (cVar2.c(cVar) != -1) {
            return true;
        }
        Iterator it = cVar2.f.iterator();
        while (it.hasNext()) {
            if (((c) it.next()).c(cVar) != -1) {
                return true;
            }
        }
        return false;
    }

    /* JADX WARN: Removed duplicated region for block: B:16:0x0032  */
    /* JADX WARN: Removed duplicated region for block: B:19:0x003b  */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public void a(com.android.tools.r8.ir.regalloc.e r6) {
        /*
            r5 = this;
            java.util.List r0 = r5.i
            int r0 = r0.size()
            r1 = 1
            if (r0 <= 0) goto L3b
            java.util.List r0 = r5.i
            int r2 = r0.size()
            int r2 = r2 - r1
            java.lang.Object r0 = r0.get(r2)
            com.android.tools.r8.ir.regalloc.e r0 = (com.android.tools.r8.ir.regalloc.e) r0
            r0.getClass()
            com.android.tools.r8.ir.regalloc.e r2 = com.android.tools.r8.ir.regalloc.e.d
            if (r0 != r2) goto L1e
            goto L32
        L1e:
            int r2 = r6.b
            int r3 = r2 % 2
            if (r3 != 0) goto L25
            goto L27
        L25:
            int r2 = r2 + 1
        L27:
            int r3 = r0.c
            int r4 = r3 % 2
            if (r4 != 0) goto L2e
            goto L30
        L2e:
            int r3 = r3 + 1
        L30:
            if (r3 <= r2) goto L34
        L32:
            r1 = 0
            goto L40
        L34:
            if (r3 != r2) goto L3b
            int r6 = r6.c
            r0.c = r6
            goto L40
        L3b:
            java.util.List r0 = r5.i
            r0.add(r6)
        L40:
            boolean r6 = com.android.tools.r8.ir.regalloc.c.s
            if (r6 != 0) goto L4d
            if (r1 == 0) goto L47
            goto L4d
        L47:
            java.lang.AssertionError r6 = new java.lang.AssertionError
            r6.<init>()
            throw r6
        L4d:
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: com.android.tools.r8.ir.regalloc.c.a(com.android.tools.r8.ir.regalloc.e):void");
    }

    public final int c(c cVar) {
        int i;
        e eVar;
        Iterator it = cVar.i.iterator();
        e eVar2 = (e) it.next();
        Iterator it2 = this.i.iterator();
        do {
            i = -1;
            if (!it2.hasNext()) {
                break;
            }
            eVar = (e) it2.next();
            while (eVar2.c <= eVar.b) {
                if (!it.hasNext()) {
                    return -1;
                }
                eVar2 = (e) it.next();
            }
            i = eVar2.b;
        } while (i >= eVar.c);
        return i;
    }

    public c c(int i) {
        int iBinarySearch;
        if (!s && this.e != this) {
            throw new AssertionError();
        }
        if (e() <= i && a() > i) {
            return this;
        }
        c cVar = a() == i ? this : null;
        if (this.f.size() <= 100) {
            iBinarySearch = 0;
        } else {
            l();
            iBinarySearch = Collections.binarySearch(this.g, Integer.valueOf(i));
            if (iBinarySearch < 0) {
                iBinarySearch = -(iBinarySearch + 1);
            }
        }
        while (iBinarySearch < this.f.size()) {
            c cVar2 = (c) this.f.get(iBinarySearch);
            if (cVar2.e() <= i && cVar2.a() > i) {
                return cVar2;
            }
            if (cVar2.a() == i) {
                cVar = cVar2;
            }
            iBinarySearch++;
        }
        if (cVar != null) {
            return cVar;
        }
        if (s) {
            return null;
        }
        throw new AssertionError("Couldn't find split covering instruction position.");
    }

    public void a(d dVar) {
        this.j.add(dVar);
        this.p = Math.min(this.p, dVar.c);
    }

    public int a() {
        if (!s && this.i.isEmpty()) {
            throw new AssertionError();
        }
        return ((e) this.i.get(r0.size() - 1)).c;
    }

    public c(c cVar) {
        this.o = false;
        this.e = cVar;
        this.b = cVar.b;
        this.o = cVar.o;
    }

    public final boolean a(int i, boolean z) {
        if (this.l == i) {
            return true;
        }
        if (this.b.Y().b() && this.l + 1 == i) {
            return true;
        }
        return z && this.l == i + 1;
    }

    public final void a(c cVar, PriorityQueue priorityQueue) {
        if (c(cVar) != -1) {
            return;
        }
        boolean zRemove = priorityQueue.remove(this);
        this.m = Integer.valueOf(cVar.l);
        if (zRemove) {
            priorityQueue.add(this);
        }
    }

    public final void a(IntConsumer intConsumer) {
        if (!s && this.l == Integer.MIN_VALUE) {
            throw new AssertionError();
        }
        intConsumer.accept(this.l);
        if (this.b.Y().b()) {
            intConsumer.accept(this.l + 1);
        }
    }

    public final boolean g() {
        return this.f.size() != 0;
    }
}
