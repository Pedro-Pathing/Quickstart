package com.android.tools.r8.ir.regalloc;

import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.AbstractC1549dg;
import com.android.tools.r8.internal.AbstractC2584oX;
import com.android.tools.r8.internal.AbstractC3250vj0;
import com.android.tools.r8.internal.C1190Zz;
import com.android.tools.r8.internal.C1397c3;
import com.android.tools.r8.internal.C1762fq;
import com.android.tools.r8.internal.C2026ig;
import com.android.tools.r8.internal.C3126uP;
import com.android.tools.r8.internal.InterfaceC1160Yw;
import com.android.tools.r8.internal.Nk0;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.TreeSet;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public class h {
    public static final /* synthetic */ boolean g = true;
    public final TreeSet a;
    public final HashMap b;
    public int c;
    public final InterfaceC1160Yw d;
    public final AbstractC2584oX e;
    public final int f;

    public h(InterfaceC1160Yw interfaceC1160Yw, int i, AbstractC2584oX abstractC2584oX) {
        this.a = new TreeSet();
        this.b = new HashMap();
        this.c = 0;
        this.d = interfaceC1160Yw;
        this.f = i;
        this.e = abstractC2584oX;
    }

    public void a(g gVar) {
        this.a.add(gVar);
        int i = gVar.d;
        if (i != Integer.MIN_VALUE) {
            this.b.put(Integer.valueOf(i), Integer.valueOf(gVar.d));
        }
        this.b.put(Integer.valueOf(gVar.c), Integer.valueOf(gVar.c));
    }

    public void a() {
        AbstractC1076Vw c3126uP;
        if (!g) {
            C1190Zz c1190Zz = new C1190Zz(this.a.size());
            Iterator it = this.a.iterator();
            while (it.hasNext()) {
                boolean zAdd = c1190Zz.add(((g) it.next()).c);
                if (!g && !zAdd) {
                    throw new AssertionError();
                }
            }
        }
        LinkedList linkedList = new LinkedList();
        Iterator it2 = this.a.iterator();
        while (it2.hasNext()) {
            g gVar = (g) it2.next();
            if (!gVar.a(this.a, this.b)) {
                linkedList.addLast(gVar);
                it2.remove();
            }
        }
        while (true) {
            if (linkedList.isEmpty() && this.a.isEmpty()) {
                return;
            }
            while (!linkedList.isEmpty()) {
                g gVar2 = (g) linkedList.removeFirst();
                boolean z = g;
                if (!z && gVar2.a(this.a, this.b)) {
                    throw new AssertionError();
                }
                AbstractC1076Vw abstractC1076Vw = gVar2.e;
                if (abstractC1076Vw != null) {
                    if (abstractC1076Vw.l1()) {
                        C1397c3 c1397c3T = gVar2.e.t();
                        c3126uP = new C3126uP(new C1762fq(gVar2.c, c1397c3T.a()), new C1762fq(c1397c3T.d().j.l, c1397c3T.a()));
                    } else {
                        if (!z && !gVar2.e.r2()) {
                            throw new AssertionError();
                        }
                        AbstractC1549dg abstractC1549dgY0 = gVar2.e.Y0();
                        if (abstractC1549dgY0.A1()) {
                            c3126uP = new C2026ig(new C1762fq(gVar2.c, gVar2.e.a()), abstractC1549dgY0.H().P2());
                        } else {
                            throw new Nk0("Unexpected definition");
                        }
                    }
                } else {
                    c3126uP = new C3126uP(new C1762fq(gVar2.c, gVar2.b), new C1762fq(((Integer) this.b.get(Integer.valueOf(gVar2.d))).intValue(), gVar2.b));
                }
                c3126uP.b(this.e);
                this.d.add(c3126uP);
                Integer numValueOf = Integer.valueOf(gVar2.c);
                int i = gVar2.d;
                if (i != Integer.MIN_VALUE) {
                    this.b.put(Integer.valueOf(i), numValueOf);
                }
                Iterator it3 = this.a.iterator();
                while (it3.hasNext()) {
                    g gVar3 = (g) it3.next();
                    if (!gVar3.a(this.a, this.b)) {
                        linkedList.addLast(gVar3);
                        it3.remove();
                    }
                }
            }
            if (!this.a.isEmpty()) {
                Iterator it4 = this.a.iterator();
                g gVar4 = null;
                while (it4.hasNext()) {
                    gVar4 = (g) it4.next();
                    if (!gVar4.b.M()) {
                        break;
                    }
                }
                it4.remove();
                int i2 = gVar4.c;
                AbstractC3250vj0 abstractC3250vj0 = gVar4.b;
                ArrayList<g> arrayList = new ArrayList();
                if (!g && i2 == Integer.MIN_VALUE) {
                    throw new AssertionError();
                }
                for (g gVar5 : this.a) {
                    int i3 = gVar5.d;
                    if (i3 != Integer.MIN_VALUE) {
                        int iIntValue = ((Integer) this.b.get(Integer.valueOf(i3))).intValue();
                        if (iIntValue == i2) {
                            arrayList.add(gVar5);
                        } else if (gVar5.b.M() && iIntValue + 1 == i2) {
                            arrayList.add(gVar5);
                        } else if (abstractC3250vj0.M() && iIntValue - 1 == i2) {
                            arrayList.add(gVar5);
                        }
                    }
                }
                if (!g && arrayList.size() <= 0) {
                    throw new AssertionError();
                }
                for (g gVar6 : arrayList) {
                    C3126uP c3126uP2 = new C3126uP(new C1762fq(this.f + this.c, gVar6.b), new C1762fq(((Integer) this.b.get(Integer.valueOf(gVar6.d))).intValue(), gVar6.b));
                    c3126uP2.b(this.e);
                    this.d.add(c3126uP2);
                    this.b.put(Integer.valueOf(gVar6.d), Integer.valueOf(this.f + this.c));
                    this.c = gVar6.b.O() + this.c;
                }
                linkedList.addLast(gVar4);
            }
        }
    }

    public h(InterfaceC1160Yw interfaceC1160Yw, int i) {
        this(interfaceC1160Yw, i, AbstractC2584oX.r());
    }
}
