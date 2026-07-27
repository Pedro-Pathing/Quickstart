package com.android.tools.r8.ir.optimize.info;

import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.C0257g1;
import com.android.tools.r8.graph.C0266h3;
import com.android.tools.r8.graph.C0291j1;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.C0427y5;
import com.android.tools.r8.graph.F0;
import com.android.tools.r8.graph.InterfaceC0227d1;
import com.android.tools.r8.graph.InterfaceC0434z5;
import com.android.tools.r8.internal.AbstractC0564Em;
import com.android.tools.r8.internal.AbstractC0575Ew;
import com.android.tools.r8.internal.AbstractC0722Jn;
import com.android.tools.r8.internal.B1;
import com.android.tools.r8.internal.B7;
import com.android.tools.r8.internal.C2037il0;
import com.android.tools.r8.internal.C2789qk0;
import com.android.tools.r8.internal.InterfaceC2591oc;
import com.android.tools.r8.internal.Jb0;
import com.android.tools.r8.internal.Zf0;
import com.android.tools.r8.ir.optimize.O;
import com.android.tools.r8.shaking.AbstractC3897l1;
import com.android.tools.r8.shaking.C3877i;
import com.android.tools.r8.shaking.C3884j;
import java.util.BitSet;
import java.util.IdentityHashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Predicate;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class z extends y {
    public static final /* synthetic */ boolean f = true;
    public final C3884j b;
    public final IdentityHashMap c;
    public final IdentityHashMap d;
    public final IdentityHashMap e;

    public z() {
        boolean z = C3877i.J;
        this.b = new C3884j();
        this.c = new IdentityHashMap();
        this.d = new IdentityHashMap();
        this.e = new IdentityHashMap();
    }

    public final synchronized v a(C0257g1 c0257g1) {
        v vVar = (v) this.c.get(c0257g1);
        if (vVar != null) {
            return vVar;
        }
        v vVar2 = (v) c0257g1.l.b();
        vVar2.getClass();
        v vVar3 = new v();
        vVar3.a = vVar2.a;
        vVar3.b = vVar2.b;
        vVar3.c = vVar2.c;
        vVar3.d = vVar2.d;
        this.c.put(c0257g1, vVar3);
        return vVar3;
    }

    @Override // com.android.tools.r8.internal.BO
    public final void b(A5 a5, Jb0 jb0) {
        f(a5.e()).p = jb0;
    }

    @Override // com.android.tools.r8.internal.BO
    public final synchronized void c(A5 a5) {
        f(a5.e()).u |= 32;
    }

    @Override // com.android.tools.r8.internal.BO
    public final synchronized void d(C0291j1 c0291j1) {
        f(c0291j1).u |= 16;
    }

    @Override // com.android.tools.r8.internal.BO
    public final synchronized void e(C0291j1 c0291j1) {
        f(c0291j1).u |= 2;
    }

    public final synchronized w f(C0291j1 c0291j1) {
        w wVar = (w) this.d.get(c0291j1);
        if (wVar != null) {
            return wVar;
        }
        c0291j1.P0();
        w wVar2 = (w) c0291j1.m.b();
        wVar2.getClass();
        w wVar3 = new w(wVar2);
        this.d.put(c0291j1, wVar3);
        return wVar3;
    }

    @Override // com.android.tools.r8.internal.BO
    public final void b(A5 a5, BitSet bitSet) {
        w wVarF = f(a5.e());
        if (!bitSet.isEmpty()) {
            wVarF.s = bitSet;
        } else {
            wVarF.s = null;
        }
    }

    public final void c() {
        Iterator it = this.d.entrySet().iterator();
        Predicate predicate = new Predicate() { // from class: com.android.tools.r8.ir.optimize.info.z$$ExternalSyntheticLambda0
            @Override // java.util.function.Predicate
            public final boolean test(Object obj) {
                return z.a((Map.Entry) obj);
            }
        };
        while (it.hasNext()) {
            if (predicate.test(it.next())) {
                it.remove();
            }
        }
        Iterator it2 = this.e.entrySet().iterator();
        Predicate predicate2 = new Predicate() { // from class: com.android.tools.r8.ir.optimize.info.z$$ExternalSyntheticLambda1
            @Override // java.util.function.Predicate
            public final boolean test(Object obj) {
                return z.b((Map.Entry) obj);
            }
        };
        while (it2.hasNext()) {
            if (predicate2.test(it2.next())) {
                it2.remove();
            }
        }
        this.c.forEach(new BiConsumer() { // from class: com.android.tools.r8.ir.optimize.info.z$$ExternalSyntheticLambda2
            @Override // java.util.function.BiConsumer
            public final void accept(Object obj, Object obj2) {
                ((C0257g1) obj).a((v) obj2);
            }
        });
        this.c.clear();
        this.d.forEach(new BiConsumer() { // from class: com.android.tools.r8.ir.optimize.info.z$$ExternalSyntheticLambda3
            @Override // java.util.function.BiConsumer
            public final void accept(Object obj, Object obj2) {
                ((C0291j1) obj).a((w) obj2);
            }
        });
        this.d.clear();
        this.e.forEach(new BiConsumer() { // from class: com.android.tools.r8.ir.optimize.info.z$$ExternalSyntheticLambda4
            @Override // java.util.function.BiConsumer
            public final void accept(Object obj, Object obj2) {
                ((C0291j1) obj).a((O) obj2);
            }
        });
        this.e.clear();
    }

    @Override // com.android.tools.r8.internal.BO
    public final synchronized void b(A5 a5) {
        f(a5.e()).v = 3;
    }

    public static boolean b(Map.Entry entry) {
        return ((C0291j1) entry.getKey()).t;
    }

    @Override // com.android.tools.r8.internal.BO
    public final synchronized void a(A5 a5, B7 b7) {
        f(a5.e()).l = b7;
    }

    @Override // com.android.tools.r8.internal.BO
    public final synchronized void a(A5 a5, InterfaceC2591oc interfaceC2591oc) {
        f(a5.e()).g = interfaceC2591oc;
    }

    public final void b() {
        boolean z = f;
        if (!z && !this.b.a.isEmpty()) {
            throw new AssertionError();
        }
        if (!z && !this.c.isEmpty()) {
            throw new AssertionError(Zf0.a(", ", this.c.keySet()));
        }
        if (!z && !this.d.isEmpty()) {
            throw new AssertionError(Zf0.a(", ", this.d.keySet()));
        }
        if (!z && !this.e.isEmpty()) {
            throw new AssertionError(Zf0.a(", ", this.e.keySet()));
        }
    }

    @Override // com.android.tools.r8.internal.BO
    public final synchronized void a(A5 a5, AbstractC0722Jn abstractC0722Jn) {
        f(a5.e()).a(abstractC0722Jn);
    }

    @Override // com.android.tools.r8.internal.BO
    public final synchronized void a(A5 a5, Jb0 jb0) {
        f(a5.e()).q = jb0;
    }

    @Override // com.android.tools.r8.internal.BO
    public final synchronized void a(A5 a5, BitSet bitSet) {
        f(a5.e()).a(bitSet);
    }

    @Override // com.android.tools.r8.internal.BO
    public final synchronized void a(A5 a5) {
        f(a5.e()).i = C2789qk0.a;
    }

    @Override // com.android.tools.r8.internal.BO
    public final synchronized void b(C0291j1 c0291j1) {
        f(c0291j1).u &= -9;
    }

    @Override // com.android.tools.r8.ir.optimize.info.y
    public final void a(Consumer consumer) {
        consumer.accept(this.b);
    }

    public static boolean a(Map.Entry entry) {
        return ((C0291j1) entry.getKey()).t;
    }

    @Override // com.android.tools.r8.internal.BO
    public final synchronized void b(C0291j1 c0291j1, BitSet bitSet) {
        f(c0291j1).o = bitSet;
    }

    @Override // com.android.tools.r8.internal.BO
    public final synchronized void c(C0291j1 c0291j1) {
        f(c0291j1).u |= 4;
    }

    @Override // com.android.tools.r8.internal.InterfaceC3551yp
    public final void a(C0257g1 c0257g1, AbstractC0564Em abstractC0564Em) {
        a(c0257g1).d = abstractC0564Em;
    }

    @Override // com.android.tools.r8.internal.InterfaceC3551yp
    public final void a(C0257g1 c0257g1, int i) {
        v vVarA = a(c0257g1);
        vVarA.c = i | vVarA.c;
    }

    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.android.tools.r8.internal.InterfaceC3551yp
    public final void a(C0257g1 c0257g1, C0421y c0421y, B1 b1) {
        boolean z = f;
        if (!z) {
            C0266h3 c0266h3 = ((C3877i) c0421y.g()).s;
            if (!c0266h3.a.containsKey(c0257g1.getReference())) {
                throw new AssertionError();
            }
        }
        if (!z && ((C3877i) c0421y.g()).s.a(c0257g1.getReference()).d()) {
            throw new AssertionError();
        }
        if (!z) {
            InterfaceC0434z5 interfaceC0434z5B = c0257g1.b(c0421y);
            if (!y.a) {
                c0421y.getClass();
                ((F0) interfaceC0434z5B).f0();
                if (!c0421y.a(((C0427y5) interfaceC0434z5B).O()).a(c0421y, interfaceC0434z5B)) {
                    throw new AssertionError();
                }
            }
        }
        a(c0257g1).a(b1, c0257g1);
    }

    @Override // com.android.tools.r8.internal.BO
    public final synchronized void a(Set set, C0291j1 c0291j1) {
        w wVarF = f(c0291j1);
        if (set.isEmpty()) {
            wVarF.c = C3682d.c;
        } else {
            wVarF.c = set;
        }
    }

    @Override // com.android.tools.r8.internal.BO
    public final synchronized void a(C0291j1 c0291j1, int i) {
        f(c0291j1).b(i);
    }

    @Override // com.android.tools.r8.internal.BO
    public final synchronized void a(C0291j1 c0291j1, C0421y c0421y, B1 b1) {
        AbstractC3897l1 abstractC3897l1A;
        if (!f) {
            InterfaceC0434z5 interfaceC0434z5A = c0291j1.a((InterfaceC0227d1) c0421y);
            if (!y.a) {
                if (interfaceC0434z5A.f0()) {
                    abstractC3897l1A = c0421y.a(interfaceC0434z5A.O());
                } else {
                    abstractC3897l1A = c0421y.a(((A5) interfaceC0434z5A).c0());
                }
                if (!abstractC3897l1A.a(c0421y, interfaceC0434z5A)) {
                    throw new AssertionError();
                }
            }
        }
        f(c0291j1).a(b1, c0291j1);
    }

    @Override // com.android.tools.r8.internal.BO
    public final void a(C0291j1 c0291j1, C2037il0 c2037il0) {
        f(c0291j1).e = c2037il0;
    }

    @Override // com.android.tools.r8.internal.BO
    public final synchronized void a(C0421y c0421y, C0291j1 c0291j1, AbstractC0564Em abstractC0564Em) {
        f(c0291j1).a(c0421y, c0291j1, abstractC0564Em);
    }

    @Override // com.android.tools.r8.internal.BO
    public final synchronized void a(C0291j1 c0291j1, O o) {
        this.e.put(c0291j1, o);
    }

    @Override // com.android.tools.r8.internal.BO
    public final synchronized void a(C0291j1 c0291j1, AbstractC0575Ew abstractC0575Ew) {
        f(c0291j1).m = abstractC0575Ew;
    }

    @Override // com.android.tools.r8.internal.BO
    public final synchronized void a(C0291j1 c0291j1) {
        f(c0291j1).u |= 128;
    }

    @Override // com.android.tools.r8.internal.BO
    public final synchronized void a(C0291j1 c0291j1, BitSet bitSet) {
        f(c0291j1).n = bitSet;
    }
}
