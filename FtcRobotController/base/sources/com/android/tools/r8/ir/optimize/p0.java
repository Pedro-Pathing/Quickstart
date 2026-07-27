package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.C0291j1;
import com.android.tools.r8.graph.C0309l1;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.C0427y5;
import com.android.tools.r8.graph.I2;
import com.android.tools.r8.internal.AbstractC0548Dw;
import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.Al0$$ExternalSyntheticLambda0;
import com.android.tools.r8.internal.B1;
import com.android.tools.r8.internal.Bc0;
import com.android.tools.r8.internal.C0756Kt;
import com.android.tools.r8.internal.C0851Nw;
import com.android.tools.r8.internal.C0874Ot;
import com.android.tools.r8.internal.C0956Rv;
import com.android.tools.r8.internal.C0996Tg;
import com.android.tools.r8.internal.C3147ue0;
import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.internal.C3239ve0;
import com.android.tools.r8.internal.C3654zw;
import com.android.tools.r8.internal.H5;
import com.android.tools.r8.internal.InterfaceC2803qw;
import com.android.tools.r8.internal.JM;
import com.android.tools.r8.internal.K5;
import com.android.tools.r8.internal.Nk0;
import com.android.tools.r8.internal.PX;
import com.android.tools.r8.internal.SW;
import com.android.tools.r8.internal.TU;
import com.android.tools.r8.internal.WB;
import com.android.tools.r8.shaking.C3877i;
import java.util.IdentityHashMap;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.Predicate;
import java.util.function.Supplier;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class p0 {
    public static final /* synthetic */ boolean j = true;
    public final A5 a;
    public final C0874Ot b;
    public final int c;
    public C3685j0 f;
    public final /* synthetic */ q0 i;
    public final C3658a d = new C3658a();
    public final k0 e = new k0();
    public final IdentityHashMap g = new IdentityHashMap();
    public boolean h = false;

    public p0(q0 q0Var, C0874Ot c0874Ot) {
        this.i = q0Var;
        this.a = c0874Ot.i();
        this.b = c0874Ot;
        this.c = Math.max(50, 10000 / c0874Ot.d.size());
        if (!j && q0Var.a.M().Z0) {
            throw new AssertionError();
        }
    }

    public static /* synthetic */ void a(List list, C3161ul0 c3161ul0) {
        if (c3161ul0.l()) {
            list.add(c3161ul0.p());
        }
    }

    public final void b() {
        this.g.forEach(new BiConsumer() { // from class: com.android.tools.r8.ir.optimize.p0$$ExternalSyntheticLambda1
            @Override // java.util.function.BiConsumer
            public final void accept(Object obj, Object obj2) {
                this.f$0.a((H5) obj, (Set) obj2);
            }
        });
    }

    /* JADX WARN: Removed duplicated region for block: B:66:0x01b1  */
    /* JADX WARN: Type inference fix 'apply assigned field type' failed
    java.lang.UnsupportedOperationException: ArgType.getObject(), call class: class jadx.core.dex.instructions.args.ArgType$UnknownArg
    	at jadx.core.dex.instructions.args.ArgType.getObject(ArgType.java:593)
    	at jadx.core.dex.attributes.nodes.ClassTypeVarsAttr.getTypeVarsMapFor(ClassTypeVarsAttr.java:35)
    	at jadx.core.dex.nodes.utils.TypeUtils.replaceClassGenerics(TypeUtils.java:177)
    	at jadx.core.dex.visitors.typeinference.FixTypesVisitor.insertExplicitUseCast(FixTypesVisitor.java:397)
    	at jadx.core.dex.visitors.typeinference.FixTypesVisitor.tryFieldTypeWithNewCasts(FixTypesVisitor.java:359)
    	at jadx.core.dex.visitors.typeinference.FixTypesVisitor.applyFieldType(FixTypesVisitor.java:309)
    	at jadx.core.dex.visitors.typeinference.FixTypesVisitor.visit(FixTypesVisitor.java:94)
     */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public final com.android.tools.r8.internal.C0669Id c() {
        /*
            Method dump skipped, instruction units count: 1506
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.android.tools.r8.ir.optimize.p0.c():com.android.tools.r8.internal.Id");
    }

    public final void b(I2 i2) {
        C0956Rv c0956Rv = this.f.g;
        if (c0956Rv == null || c0956Rv.i != i2) {
            return;
        }
        ((Set) this.g.computeIfAbsent(c0956Rv.b(), JM.a(new Al0$$ExternalSyntheticLambda0()))).add(c0956Rv);
    }

    public final void a(SW sw) {
        sw.a((C0756Kt) null, this.d, C0996Tg.b(), PX.c);
    }

    public final /* synthetic */ void a(final H5 h5, Set set) {
        if (!j && !set.stream().allMatch(new Predicate() { // from class: com.android.tools.r8.ir.optimize.p0$$ExternalSyntheticLambda6
            @Override // java.util.function.Predicate
            public final boolean test(Object obj) {
                return p0.a(h5, (AbstractC1076Vw) obj);
            }
        })) {
            throw new AssertionError();
        }
        K5 k5A = h5.a(this.b);
        while (k5A.hasNext()) {
            AbstractC1076Vw next = k5A.next();
            if (!j && next.d2()) {
                throw new AssertionError();
            }
            if (set.contains(next)) {
                k5A.p();
                this.h = true;
                set.remove(next);
                if (set.isEmpty()) {
                    return;
                }
            }
        }
    }

    public static /* synthetic */ boolean a(H5 h5, AbstractC1076Vw abstractC1076Vw) {
        return abstractC1076Vw.b() == h5;
    }

    public final void a(final WB wb) {
        if (!this.i.a.g().i()) {
            a();
            return;
        }
        final C0421y<C3877i> c0421yV = this.i.a.V();
        com.android.tools.r8.graph.H0 h0G = wb.g(this.i.a, this.a);
        if (h0G != null && h0G.e().r1()) {
            C0291j1 c0291j1E = h0G.e();
            c0291j1E.P0();
            AbstractC0548Dw abstractC0548DwA = c0291j1E.m.a(wb);
            if (abstractC0548DwA.e()) {
                a();
            }
            abstractC0548DwA.b().b(this.i.a, new BiConsumer() { // from class: com.android.tools.r8.ir.optimize.p0$$ExternalSyntheticLambda3
                @Override // java.util.function.BiConsumer
                public final void accept(Object obj, Object obj2) {
                    this.f$0.a(c0421yV, wb, (com.android.tools.r8.graph.F0) obj, (InterfaceC2803qw) obj2);
                }
            });
            return;
        }
        a();
    }

    public final void a(C0421y c0421y, WB wb, com.android.tools.r8.graph.F0 f0, InterfaceC2803qw interfaceC2803qw) {
        f0.getClass();
        if (f0 instanceof C0427y5) {
            C0427y5 c0427y5O = f0.O();
            if (!this.i.a.a(c0427y5O).a(c0421y, c0427y5O)) {
                return;
            }
        }
        if (interfaceC2803qw.h()) {
            C3161ul0 c3161ul0B = wb.b(interfaceC2803qw.f().a);
            l0 l0Var = new l0(f0.getReference(), wb.V2().i());
            if (f0.b(c0421y)) {
                C3685j0 c3685j0 = this.f;
                n0 n0Var = new n0(this, c3161ul0B);
                c3685j0.c();
                if (c3685j0.b == null) {
                    c3685j0.b = new LinkedHashMap();
                }
                c3685j0.b.put(l0Var, n0Var);
                return;
            }
            this.f.a(l0Var, new n0(this, c3161ul0B));
            return;
        }
        if (interfaceC2803qw.g()) {
            Bc0 bc0J = interfaceC2803qw.j();
            if (bc0J.Q() && bc0J.b(c0421y, this.a)) {
                l0 l0Var2 = new l0(f0.getReference(), wb.V2().i());
                if (f0.b(c0421y)) {
                    C3685j0 c3685j02 = this.f;
                    o0 o0Var = new o0(this, bc0J);
                    c3685j02.c();
                    if (c3685j02.b == null) {
                        c3685j02.b = new LinkedHashMap();
                    }
                    c3685j02.b.put(l0Var2, o0Var);
                    return;
                }
                this.f.a(l0Var2, new o0(this, bc0J));
                return;
            }
            return;
        }
        if (!j && !interfaceC2803qw.a()) {
            throw new AssertionError();
        }
    }

    public final boolean a(I2 i2) {
        C3685j0 c3685j0 = this.f;
        c3685j0.c();
        if (c3685j0.d == null) {
            c3685j0.d = new LinkedHashSet();
        }
        return c3685j0.d.add(i2);
    }

    public final void a(C0851Nw c0851Nw, com.android.tools.r8.graph.F0 f0) {
        this.f.b(f0.getReference());
        if (c0851Nw.a(this.i.a, this.a)) {
            this.f.a();
        }
        l0 l0Var = new l0(f0.getReference(), c0851Nw.h().i());
        n0 n0Var = new n0(this, c0851Nw.value());
        if (f0.b(this.i.a)) {
            if (!j && f0.e().g.f() && !this.a.e().r1()) {
                throw new AssertionError();
            }
            C3685j0 c3685j0 = this.f;
            c3685j0.c();
            if (c3685j0.b == null) {
                c3685j0.b = new LinkedHashMap();
            }
            c3685j0.b.put(l0Var, n0Var);
        } else {
            this.f.a(l0Var, n0Var);
        }
        C3685j0 c3685j02 = this.f;
        c3685j02.c();
        if (c3685j02.h == null) {
            c3685j02.h = new LinkedHashMap();
        }
        C0851Nw c0851Nw2 = (C0851Nw) c3685j02.h.put(l0Var, c0851Nw);
        if (c0851Nw2 != null) {
            ((Set) this.g.computeIfAbsent(c0851Nw2.b(), JM.a(new Al0$$ExternalSyntheticLambda0()))).add(c0851Nw2);
        }
        C3685j0 c3685j03 = this.f;
        C0956Rv c0956Rv = c3685j03.g;
        c3685j03.g = null;
    }

    public final void a(C3239ve0 c3239ve0, final com.android.tools.r8.graph.F0 f0) {
        a(f0.s());
        a(c3239ve0);
        if (c3239ve0.a(this.i.a, this.a)) {
            this.f.a();
        }
        n0 n0Var = new n0(this, c3239ve0.value());
        if (f0.b(this.i.a)) {
            if (!j && !this.i.a.a(new Supplier() { // from class: com.android.tools.r8.ir.optimize.p0$$ExternalSyntheticLambda0
                @Override // java.util.function.Supplier
                public final Object get() {
                    return this.f$0.a(f0);
                }
            })) {
                throw new AssertionError();
            }
            this.f.a(f0.getReference(), n0Var);
        } else {
            this.f.b(f0.getReference(), n0Var);
            C3685j0 c3685j0 = this.f;
            C0309l1 reference = f0.getReference();
            c3685j0.c();
            if (c3685j0.i == null) {
                c3685j0.i = new LinkedHashMap();
            }
            C3239ve0 c3239ve02 = (C3239ve0) c3685j0.i.put(reference, c3239ve0);
            if (c3239ve02 != null) {
                ((Set) this.g.computeIfAbsent(c3239ve02.b(), JM.a(new Al0$$ExternalSyntheticLambda0()))).add(c3239ve02);
            }
        }
        b(f0.s());
        C3685j0 c3685j02 = this.f;
        C0956Rv c0956Rv = c3685j02.g;
        c3685j02.g = null;
    }

    public final Boolean a(com.android.tools.r8.graph.F0 f0) {
        return Boolean.valueOf(!f0.e().g.f() || this.a.e().n1());
    }

    public final void a(final C3161ul0 c3161ul0, TU tu) {
        final C0421y<C3877i> c0421yV = this.i.a.V();
        tu.a(new BiConsumer() { // from class: com.android.tools.r8.ir.optimize.p0$$ExternalSyntheticLambda2
            @Override // java.util.function.BiConsumer
            public final void accept(Object obj, Object obj2) {
                this.f$0.a(c0421yV, c3161ul0, (C0309l1) obj, (B1) obj2);
            }
        });
    }

    public final void a(C0421y c0421y, C3161ul0 c3161ul0, C0309l1 c0309l1, B1 b1) {
        if (b1.g()) {
            com.android.tools.r8.graph.F0 f0A = c0421y.a(c0309l1);
            boolean z = C0427y5.f;
            C0427y5 c0427y5O = f0A != null ? f0A.O() : null;
            if (c0427y5O == null || this.i.a.a(c0427y5O).a(c0421y, c0427y5O)) {
                Bc0 bc0J = b1.j();
                if (bc0J.Q() && bc0J.b(c0421y, this.a)) {
                    C3685j0 c3685j0 = this.f;
                    l0 l0Var = new l0(c0309l1, c3161ul0);
                    o0 o0Var = new o0(this, bc0J);
                    c3685j0.c();
                    if (c3685j0.b == null) {
                        c3685j0.b = new LinkedHashMap();
                    }
                    c3685j0.b.put(l0Var, o0Var);
                }
            }
        }
    }

    public final void a() {
        C3685j0 c3685j0 = this.f;
        c3685j0.a = null;
        c3685j0.e = null;
        c3685j0.f = null;
        c3685j0.a();
        C3685j0 c3685j02 = this.f;
        C0956Rv c0956Rv = c3685j02.g;
        c3685j02.g = null;
    }

    public final void a(AbstractC1076Vw abstractC1076Vw) {
        if (!j && !abstractC1076Vw.N1() && !abstractC1076Vw.w2()) {
            throw new AssertionError();
        }
        if (abstractC1076Vw.e()) {
            if (abstractC1076Vw.d(this.i.a, this.a)) {
                C3685j0 c3685j0 = this.f;
                c3685j0.f = null;
                c3685j0.a();
                return;
            } else {
                C3685j0 c3685j02 = this.f;
                C0309l1 field = abstractC1076Vw.J0().getField();
                LinkedHashMap linkedHashMap = c3685j02.f;
                if (linkedHashMap != null) {
                    linkedHashMap.remove(field);
                    return;
                }
                return;
            }
        }
        if (!abstractC1076Vw.N1() && !(abstractC1076Vw instanceof C3147ue0)) {
            if (abstractC1076Vw instanceof C3654zw) {
                throw new Nk0();
            }
        } else if (abstractC1076Vw.d(this.i.a, this.a)) {
            C3685j0 c3685j03 = this.f;
            c3685j03.f = null;
            c3685j03.a();
        }
    }
}
