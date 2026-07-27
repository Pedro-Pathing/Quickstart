package com.android.tools.r8.ir.optimize.info;

import com.android.tools.r8.graph.C0291j1;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.L5;
import com.android.tools.r8.internal.AbstractC0548Dw;
import com.android.tools.r8.internal.AbstractC0564Em;
import com.android.tools.r8.internal.AbstractC0575Ew;
import com.android.tools.r8.internal.AbstractC0722Jn;
import com.android.tools.r8.internal.AbstractC1308bC;
import com.android.tools.r8.internal.AbstractC2670pV;
import com.android.tools.r8.internal.AbstractC3250vj0;
import com.android.tools.r8.internal.AbstractC3575z6;
import com.android.tools.r8.internal.AbstractC3650zs;
import com.android.tools.r8.internal.B1;
import com.android.tools.r8.internal.B7;
import com.android.tools.r8.internal.C0882Pb;
import com.android.tools.r8.internal.C1092Wm;
import com.android.tools.r8.internal.C1395c2;
import com.android.tools.r8.internal.C2700pk0;
import com.android.tools.r8.internal.C2789qk0;
import com.android.tools.r8.internal.C2867rd;
import com.android.tools.r8.internal.C3050tc0;
import com.android.tools.r8.internal.Dk0;
import com.android.tools.r8.internal.FQ;
import com.android.tools.r8.internal.InterfaceC2591oc;
import com.android.tools.r8.internal.Jb0;
import com.android.tools.r8.internal.Ob0;
import com.android.tools.r8.internal.WB;
import com.android.tools.r8.internal.Y6;
import java.util.BitSet;
import java.util.Set;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public class w extends h {
    public static final int w;
    public static final /* synthetic */ boolean x = true;
    public AbstractC3679a b;
    public Set c;
    public int d;
    public com.android.tools.r8.internal.r e;
    public B1 f;
    public InterfaceC2591oc g;
    public boolean h;
    public AbstractC0722Jn i;
    public AbstractC0564Em j;
    public AbstractC2670pV k;
    public B7 l;
    public AbstractC0575Ew m;
    public BitSet n;
    public BitSet o;
    public Jb0 p;
    public Jb0 q;
    public int r;
    public BitSet s;
    public BitSet t;
    public int u;
    public int v;

    static {
        C3682d c3682d = C3682d.b;
        w = (Y6.a(true) * 8) | Y6.a(false) | (Y6.a(false) * 2) | (Y6.a(false) * 4) | (Y6.a(false) * 16) | (Y6.a(false) * 32) | (Y6.a(false) * 128) | (Y6.a(false) * 256);
    }

    public w() {
        this.b = E.a;
        this.c = C3682d.c;
        this.d = -1;
        this.e = C2700pk0.a;
        this.f = C3682d.d;
        this.g = C1395c2.a;
        this.h = false;
        this.i = C2789qk0.a;
        this.j = AbstractC0564Em.m();
        this.v = 3;
        this.k = AbstractC2670pV.c;
        this.l = null;
        this.m = C1092Wm.a;
        this.n = null;
        this.o = null;
        FQ fq = FQ.b;
        this.p = fq;
        this.q = fq;
        this.r = 0;
        this.s = null;
        this.t = null;
        this.u = w;
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final boolean A() {
        return this.v == 1;
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final AbstractC2670pV B() {
        return this.k;
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final boolean C() {
        return a(8);
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final boolean D() {
        return a(32);
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final boolean E() {
        return a(256);
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final boolean F() {
        return a(16);
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final boolean G() {
        return this.d != -1;
    }

    public void H() {
        int i;
        if (!x && (i = this.v) != 3 && i != 2) {
            throw new AssertionError();
        }
        this.v = 2;
    }

    public final void I() {
        this.f = Dk0.a;
    }

    public final void J() {
        this.l = null;
    }

    public final void K() {
        this.u &= -3;
    }

    public final void L() {
        this.g = C1395c2.a;
    }

    public final w M() {
        return a(AbstractC0564Em.m());
    }

    public final void N() {
        this.i = C2789qk0.a;
    }

    public final void O() {
        this.v = 3;
    }

    public final void P() {
        this.c = C3682d.c;
    }

    public final void Q() {
        this.u &= -129;
    }

    public final void R() {
        this.u &= -5;
    }

    public final void S() {
        this.m = C1092Wm.a;
    }

    public final void T() {
        this.u |= 8;
    }

    public final void U() {
        this.u &= -33;
    }

    public final void V() {
        this.o = null;
    }

    public final void W() {
        this.n = null;
    }

    public final void X() {
        this.p = FQ.b;
    }

    public final void Y() {
        this.u &= -17;
    }

    public final void Z() {
        this.d = -1;
    }

    @Override // com.android.tools.r8.ir.optimize.info.g
    public final w a() {
        return this;
    }

    public final void a0() {
        this.q = FQ.b;
    }

    @Override // com.android.tools.r8.ir.optimize.info.g
    public final g b() {
        return this;
    }

    public final void b0() {
        this.t = null;
    }

    @Override // com.android.tools.r8.ir.optimize.info.g
    public final boolean d() {
        return true;
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final boolean e() {
        return a(1);
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final boolean f() {
        return a(2);
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final boolean g() {
        return this.v == 2;
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final com.android.tools.r8.internal.r h() {
        return this.e;
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final B1 i() {
        return this.f;
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final AbstractC3679a j() {
        return this.b;
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final B7 k() {
        return this.l;
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final InterfaceC2591oc l() {
        return this.g;
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final AbstractC0548Dw m() {
        return this.m.b();
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final AbstractC0564Em n() {
        return this.j;
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final AbstractC0722Jn o() {
        return this.i;
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final Set p() {
        return this.c;
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final int q() {
        return this.r;
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final BitSet r() {
        return this.o;
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final BitSet s() {
        return this.n;
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final int t() {
        return this.d;
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final Jb0 u() {
        return this.q;
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final BitSet v() {
        return this.t;
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final boolean w() {
        return a(4);
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final boolean y() {
        return this.h;
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final boolean z() {
        return a(128);
    }

    public final w a(C0421y c0421y, AbstractC3650zs abstractC3650zs, Set set) {
        AbstractC0564Em abstractC0564EmA = this.j.a(c0421y, abstractC3650zs, set);
        if (!abstractC0564EmA.f() || !abstractC0564EmA.a().n().H()) {
            return a(abstractC0564EmA);
        }
        boolean z = x;
        if (!z) {
            AbstractC0564Em abstractC0564Em = this.j;
            if (!z && !abstractC0564Em.h()) {
                throw new AssertionError();
            }
            AbstractC3250vj0 abstractC3250vj0N = abstractC0564Em.a().n();
            if (!z && !abstractC3250vj0N.w()) {
                throw new AssertionError();
            }
            C2867rd c2867rdB = abstractC3250vj0N.b();
            if (!z && !c0421y.D()) {
                throw new AssertionError();
            }
            if (!z && !c0421y.R().b(c2867rdB.Q())) {
                throw new AssertionError();
            }
        }
        return a(AbstractC0564Em.m());
    }

    public final void b(int i) {
        int i2;
        boolean z = x;
        if (!z && i < 0) {
            throw new AssertionError();
        }
        if (!z && (i2 = this.d) != -1 && i2 != i) {
            throw new AssertionError();
        }
        this.d = i;
    }

    public final boolean a(int i) {
        return (i & this.u) != 0;
    }

    public final void a(C0421y c0421y, L5 l5) {
        AbstractC3679a abstractC3679a = this.b;
        abstractC3679a.getClass();
        if (abstractC3679a instanceof C3680b) {
            this.b = l5.a(this.b.a());
        }
        B7 b7 = this.l;
        if (b7 != null) {
            this.l = l5.a(b7);
        }
        this.g = l5.a(c0421y, this.g);
        w wVarA = this.j.l() ? this : a(l5.a(this.j));
        if (!wVarA.f.isUnknown()) {
            wVarA.f = l5.a(c0421y, wVarA.f);
        }
        wVarA.i = l5.a(wVarA.i);
        AbstractC0575Ew abstractC0575EwA = wVarA.m;
        if (!l5.a.b.c()) {
            abstractC0575EwA = abstractC0575EwA.a(c0421y, l5.a.b);
        }
        wVarA.m = abstractC0575EwA;
        wVarA.o = l5.a(wVarA.o);
        wVarA.n = l5.a(wVarA.n);
        wVarA.d = l5.a(wVarA.d);
        BitSet bitSetA = l5.a(wVarA.t);
        BitSet bitSet = null;
        if (bitSetA != null && !bitSetA.isEmpty()) {
            wVarA.s = bitSetA;
        } else {
            wVarA.s = null;
        }
        Jb0 jb0A = wVarA.p;
        Ob0 ob0 = c0421y.y;
        if (!l5.a.b.c()) {
            jb0A = jb0A.a(c0421y, l5.a.b, ob0);
        }
        wVarA.p = jb0A;
        Jb0 jb0A2 = wVarA.q;
        Ob0 ob02 = c0421y.y;
        if (!l5.a.b.c()) {
            jb0A2 = jb0A2.a(c0421y, l5.a.b, ob02);
        }
        wVarA.q = jb0A2;
        BitSet bitSetA2 = l5.a(wVarA.t);
        if (bitSetA2 != null && !bitSetA2.isEmpty()) {
            bitSet = bitSetA2;
        }
        wVarA.t = bitSet;
    }

    public final w a(AbstractC0722Jn abstractC0722Jn) {
        if (!x) {
            AbstractC0722Jn abstractC0722Jn2 = this.i;
            abstractC0722Jn2.getClass();
            if (abstractC0722Jn2 instanceof C0882Pb) {
                abstractC0722Jn.getClass();
                if (!(abstractC0722Jn instanceof C0882Pb)) {
                    throw new AssertionError();
                }
            }
        }
        this.i = abstractC0722Jn;
        return this;
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final AbstractC0548Dw a(WB wb) {
        return this.m.a();
    }

    public final void a(BitSet bitSet) {
        boolean z = x;
        if (!z) {
            x();
        }
        if (!z && x()) {
            BitSet bitSet2 = this.t;
            if (!AbstractC3575z6.a) {
                BitSet bitSet3 = (BitSet) bitSet2.clone();
                bitSet3.or(bitSet);
                if (!bitSet.equals(bitSet3)) {
                    throw new AssertionError();
                }
            }
        }
        if (bitSet.isEmpty()) {
            bitSet = null;
        }
        this.t = bitSet;
    }

    public w(w wVar) {
        this.b = E.a;
        this.c = C3682d.c;
        this.d = -1;
        this.e = C2700pk0.a;
        this.f = C3682d.d;
        this.g = C1395c2.a;
        this.h = false;
        this.i = C2789qk0.a;
        this.j = AbstractC0564Em.m();
        this.v = 3;
        this.k = AbstractC2670pV.c;
        this.l = null;
        this.m = C1092Wm.a;
        this.n = null;
        this.o = null;
        FQ fq = FQ.b;
        this.p = fq;
        this.q = fq;
        this.r = 0;
        this.s = null;
        this.t = null;
        this.u = w;
        this.e = wVar.e;
        this.b = wVar.b;
        this.u = wVar.u;
        this.c = wVar.c;
        this.d = wVar.d;
        this.f = wVar.f;
        a(wVar.j);
        this.v = wVar.v;
        this.p = wVar.p;
        this.q = wVar.q;
        this.l = wVar.l;
        this.m = wVar.m;
        this.n = wVar.n;
        this.o = wVar.o;
        this.g = wVar.g;
        this.i = wVar.i;
        this.r = wVar.r;
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final boolean a(AbstractC1308bC abstractC1308bC) {
        return a(8) && !this.p.a(abstractC1308bC);
    }

    public final void a(B1 b1, C0291j1 c0291j1) {
        if (!x) {
            b1.getClass();
            if ((b1 instanceof C3050tc0) && !c0291j1.g1().U0()) {
                throw new AssertionError();
            }
        }
        a(b1);
    }

    public final void a(B1 b1) {
        if (!x && this.f.g() && !this.f.equals(b1) && (!this.f.N() || !b1.M() || !this.f.r().R().a(b1.r().R()))) {
            throw new AssertionError("return single value changed from " + this.f + " to " + b1);
        }
        this.f = b1;
    }

    public final void a(C0421y c0421y, C0291j1 c0291j1, AbstractC0564Em abstractC0564Em) {
        AbstractC3250vj0 abstractC3250vj0B = c0291j1.g1().b((C0421y<?>) c0421y);
        if (!x) {
            a(c0421y, abstractC0564Em, abstractC3250vj0B);
        }
        a(abstractC0564Em);
    }

    public final w a(AbstractC0564Em abstractC0564Em) {
        if (!x && abstractC0564Em.f() && abstractC0564Em.a().n().H()) {
            throw new AssertionError();
        }
        this.j = abstractC0564Em;
        return this;
    }

    public final void a(C0421y c0421y, AbstractC0564Em abstractC0564Em, AbstractC3250vj0 abstractC3250vj0) {
        if (c0421y.o()) {
            AbstractC3250vj0 abstractC3250vj0A = this.j.a(abstractC3250vj0);
            AbstractC3250vj0 abstractC3250vj0A2 = abstractC0564Em.a(abstractC3250vj0);
            if (!x && !abstractC3250vj0A2.b(abstractC3250vj0A, c0421y)) {
                throw new AssertionError("upper bound type changed from " + abstractC3250vj0A + " to " + abstractC3250vj0A2);
            }
        }
    }
}
