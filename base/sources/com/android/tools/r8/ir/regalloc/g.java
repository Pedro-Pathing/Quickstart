package com.android.tools.r8.ir.regalloc;

import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.AbstractC3250vj0;
import java.util.HashMap;
import java.util.Iterator;
import java.util.TreeSet;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public class g implements Comparable<g> {
    public static final /* synthetic */ boolean f = true;
    public final AbstractC3250vj0 b;
    public final int c;
    public final int d;
    public final AbstractC1076Vw e;

    public g(int i, int i2, AbstractC3250vj0 abstractC3250vj0) {
        this.c = i;
        this.d = i2;
        this.e = null;
        this.b = abstractC3250vj0;
    }

    public final boolean a(TreeSet treeSet, HashMap map) {
        Iterator it = treeSet.iterator();
        while (it.hasNext()) {
            g gVar = (g) it.next();
            int i = gVar.d;
            if (i != Integer.MIN_VALUE && gVar != this) {
                int iIntValue = ((Integer) map.get(Integer.valueOf(i))).intValue();
                if ((this.b.M() && this.c + 1 == iIntValue) || this.c == iIntValue) {
                    return true;
                }
                if (gVar.b.M()) {
                    int iIntValue2 = ((Integer) map.get(Integer.valueOf(gVar.d))).intValue() + 1;
                    if ((this.b.M() && this.c + 1 == iIntValue2) || this.c == iIntValue2) {
                        return true;
                    }
                } else {
                    continue;
                }
            }
        }
        return false;
    }

    @Override // java.lang.Comparable
    public final int compareTo(g gVar) {
        g gVar2 = gVar;
        int i = this.d - gVar2.d;
        if (i != 0) {
            return i;
        }
        int i2 = this.c - gVar2.c;
        if (i2 != 0) {
            return i2;
        }
        if (this.b.H() != gVar2.b.H()) {
            return Boolean.compare(this.b.H(), gVar2.b.H());
        }
        if (this.b.M() != gVar2.b.M()) {
            return Boolean.compare(this.b.M(), gVar2.b.M());
        }
        if (this.b.I() != gVar2.b.I()) {
            return Boolean.compare(this.b.I(), gVar2.b.I());
        }
        AbstractC1076Vw abstractC1076Vw = this.e;
        if (abstractC1076Vw == null) {
            return gVar2.e != null ? -1 : 0;
        }
        AbstractC1076Vw abstractC1076Vw2 = gVar2.e;
        if (abstractC1076Vw2 == null) {
            return 1;
        }
        return abstractC1076Vw.e - abstractC1076Vw2.e;
    }

    public final boolean equals(Object obj) {
        if (!(obj instanceof g)) {
            return false;
        }
        g gVar = (g) obj;
        return gVar.d == this.d && gVar.c == this.c && gVar.b == this.b && gVar.e == this.e;
    }

    public final int hashCode() {
        int iHashCode = (this.b.hashCode() * 5) + (this.c * 3) + this.d;
        AbstractC1076Vw abstractC1076Vw = this.e;
        return iHashCode + (abstractC1076Vw == null ? 0 : abstractC1076Vw.hashCode());
    }

    public g(int i, AbstractC3250vj0 abstractC3250vj0, AbstractC1076Vw abstractC1076Vw) {
        if (!f && !abstractC1076Vw.r2()) {
            throw new AssertionError();
        }
        this.c = i;
        this.d = Integer.MIN_VALUE;
        this.e = abstractC1076Vw;
        this.b = abstractC3250vj0;
    }
}
