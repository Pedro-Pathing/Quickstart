package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.S4;
import com.android.tools.r8.internal.AbstractC1308bC;
import com.android.tools.r8.internal.AbstractC3650zs;
import com.android.tools.r8.internal.BX;
import com.android.tools.r8.internal.C0874Ot;
import com.android.tools.r8.internal.C1411cA;
import com.android.tools.r8.internal.C1567dq;
import com.android.tools.r8.internal.C1631ec;
import com.android.tools.r8.internal.C1711fS;
import com.android.tools.r8.internal.C1958hw;
import com.android.tools.r8.internal.C2298lM;
import com.android.tools.r8.internal.C2527no;
import com.android.tools.r8.internal.C2937sJ;
import com.android.tools.r8.internal.C3437xf;
import com.android.tools.r8.internal.Fh0;
import com.android.tools.r8.internal.InterfaceC2226kh0;
import com.android.tools.r8.internal.InterfaceC3346wh0;
import com.android.tools.r8.internal.JM;
import com.android.tools.r8.internal.JO;
import com.android.tools.r8.internal.KP;
import com.android.tools.r8.internal.LP;
import com.android.tools.r8.internal.MR;
import com.android.tools.r8.internal.NP;
import com.android.tools.r8.internal.Ot$$ExternalSyntheticLambda26;
import com.android.tools.r8.internal.RY;
import com.android.tools.r8.internal.SY;
import com.android.tools.r8.internal.TY;
import com.android.tools.r8.shaking.C3877i;
import java.util.Optional;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutorService;
import java.util.function.BiConsumer;
import java.util.function.ObjIntConsumer;
import java.util.function.Predicate;
import java.util.function.Supplier;

/* JADX INFO: renamed from: com.android.tools.r8.ir.optimize.b0, reason: case insensitive filesystem */
/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class C3661b0 {
    public static final /* synthetic */ boolean d = true;
    public final C0421y a;
    public final SY b = SY.l();
    public final int[] c;

    public C3661b0(C0421y c0421y) {
        this.a = c0421y;
        this.c = c0421y.M().T().e;
    }

    public static Optional a() {
        return Optional.of(new TY(new C3437xf(new ConcurrentHashMap())));
    }

    /* JADX INFO: renamed from: b, reason: merged with bridge method [inline-methods] */
    public final void a(final A5 a5, C0874Ot c0874Ot, final JO jo) {
        S4.c<?> cVarO;
        A5 a5A;
        C2937sJ c2937sJ = new C2937sJ(new Supplier() { // from class: com.android.tools.r8.ir.optimize.b0$$ExternalSyntheticLambda0
            @Override // java.util.function.Supplier
            public final Object get() {
                return this.f$0.a(a5, jo);
            }
        });
        for (AbstractC1308bC abstractC1308bC : c0874Ot.b((Predicate) new Ot$$ExternalSyntheticLambda26())) {
            if (!abstractC1308bC.a(this.a.a()) && (cVarO = ((C3877i) this.a.g()).b(abstractC1308bC.U2(), abstractC1308bC.T2()).o()) != null && !cVarO.a(a5, this.a).b() && (a5A = com.android.tools.r8.graph.H0.a(abstractC1308bC.g(this.a, a5))) != null && jo.c().b(a5A)) {
                S sA = ((H) c2937sJ.a(c2937sJ.b)).a(c0874Ot, abstractC1308bC, cVarO, a5A, a5, C1631ec.e, new C1958hw(), C1711fS.a);
                if (sA == null || (sA instanceof V)) {
                    SY sy = this.b;
                    sy.b.put(sy.b(a5A), Optional.empty());
                } else {
                    a(a5, a5A, jo);
                }
            }
        }
    }

    public final void a(final A5 a5, final C0874Ot c0874Ot, final JO jo, Fh0 fh0) {
        if (jo.f()) {
            fh0.a("Multi caller inliner: Record call edges", new InterfaceC2226kh0() { // from class: com.android.tools.r8.ir.optimize.b0$$ExternalSyntheticLambda1
                @Override // com.android.tools.r8.internal.InterfaceC2226kh0
                public final void b() {
                    this.f$0.a(a5, c0874Ot, jo);
                }
            });
        }
    }

    public final /* synthetic */ H a(A5 a5, JO jo) {
        return new H(this.a, new C1567dq(U.b), a5, jo, Integer.MAX_VALUE);
    }

    public final void a(A5 a5, A5 a52, JO jo) {
        SY sy = this.b;
        Optional optional = (Optional) sy.a(JM.a(new Supplier() { // from class: com.android.tools.r8.ir.optimize.b0$$ExternalSyntheticLambda7
            @Override // java.util.function.Supplier
            public final Object get() {
                return C3661b0.a();
            }
        }), sy.b(a52));
        if (optional.isPresent()) {
            TY ty = (TY) optional.get();
            ty.a.add(new C2527no(RY.a, a5));
            if (ty.a.size() > this.c.length) {
                a(a52, jo, ty);
            }
        }
    }

    public final void a(A5 a5, final JO jo, TY ty) {
        ty.a(new Predicate() { // from class: com.android.tools.r8.ir.optimize.b0$$ExternalSyntheticLambda8
            @Override // java.util.function.Predicate
            public final boolean test(Object obj) {
                return ((A5) obj).D().w();
            }
        });
        final C1411cA c1411cA = new C1411cA();
        ty.a(new ObjIntConsumer() { // from class: com.android.tools.r8.ir.optimize.b0$$ExternalSyntheticLambda9
            @Override // java.util.function.ObjIntConsumer
            public final void accept(Object obj, int i) {
                C3661b0.a(jo, c1411cA, (A5) obj, i);
            }
        });
        if (c1411cA.a() > this.c.length) {
            SY sy = this.b;
            Optional optionalEmpty = Optional.empty();
            sy.b.put(sy.b(a5), optionalEmpty);
        }
    }

    public static /* synthetic */ void a(JO jo, C1411cA c1411cA, A5 a5, int i) {
        if (jo.c().a(a5)) {
            return;
        }
        c1411cA.c(i);
    }

    public final void a(BX bx, Fh0 fh0, final ExecutorService executorService) {
        fh0.a("Multi caller inliner");
        final KP kp = (KP) fh0.a("Call graph construction", new InterfaceC3346wh0() { // from class: com.android.tools.r8.ir.optimize.b0$$ExternalSyntheticLambda3
            @Override // com.android.tools.r8.internal.InterfaceC3346wh0
            public final Object get() {
                return this.f$0.a(executorService);
            }
        });
        bx.a(this.a).a.a((C2298lM) fh0.a("Needs inlining analysis", new InterfaceC3346wh0() { // from class: com.android.tools.r8.ir.optimize.b0$$ExternalSyntheticLambda4
            @Override // com.android.tools.r8.internal.InterfaceC3346wh0
            public final Object get() {
                return this.f$0.b(kp);
            }
        }));
        fh0.b();
    }

    public final KP a(ExecutorService executorService) {
        return new LP(this.a).a(executorService);
    }

    /* JADX INFO: renamed from: a, reason: merged with bridge method [inline-methods] */
    public final C2298lM b(final KP kp) {
        final AbstractC3650zs abstractC3650zsA = this.a.A();
        final C2298lM c2298lMB = C2298lM.b(abstractC3650zsA);
        this.b.forEach(new BiConsumer() { // from class: com.android.tools.r8.ir.optimize.b0$$ExternalSyntheticLambda2
            @Override // java.util.function.BiConsumer
            public final void accept(Object obj, Object obj2) {
                this.f$0.a(kp, c2298lMB, abstractC3650zsA, (A5) obj, (Optional) obj2);
            }
        });
        this.b.b.clear();
        return c2298lMB;
    }

    public final void a(KP kp, final C2298lM c2298lM, final AbstractC3650zs abstractC3650zs, A5 a5, Optional optional) {
        int i;
        if (!a5.e().s1().c() && optional.isPresent()) {
            if (a5.e().z0() || ((C3877i) this.a.g()).b(a5.a())) {
                TY ty = (TY) optional.get();
                ty.a(new Predicate() { // from class: com.android.tools.r8.ir.optimize.b0$$ExternalSyntheticLambda5
                    @Override // java.util.function.Predicate
                    public final boolean test(Object obj) {
                        return ((A5) obj).D().w();
                    }
                });
                if (ty.a.size() == 0 || ty.a.size() > this.c.length || (i = ((NP) ((MR) kp.a.get(a5.getReference()))).c.get()) < ty.a.size() || ty.a.size() < i) {
                    return;
                }
                if (a5.e().V0().k(this.c[ty.a.size() - 1]) >= 0) {
                    ty.a(new ObjIntConsumer() { // from class: com.android.tools.r8.ir.optimize.b0$$ExternalSyntheticLambda6
                        @Override // java.util.function.ObjIntConsumer
                        public final void accept(Object obj, int i2) {
                            C3661b0.a(c2298lM, abstractC3650zs, (A5) obj, i2);
                        }
                    });
                    com.android.tools.r8.ir.optimize.info.y.a().getClass();
                    com.android.tools.r8.ir.optimize.info.w wVarZ0 = a5.e().Z0();
                    int i2 = wVarZ0.v;
                    if (i2 == 3) {
                        wVarZ0.v = 1;
                    } else if (!com.android.tools.r8.ir.optimize.info.w.x && i2 != 2) {
                        throw new AssertionError();
                    }
                }
            }
        }
    }

    public static /* synthetic */ void a(C2298lM c2298lM, AbstractC3650zs abstractC3650zs, A5 a5, int i) {
        if (a5.D().w()) {
            return;
        }
        c2298lM.a(abstractC3650zs, a5);
    }
}
