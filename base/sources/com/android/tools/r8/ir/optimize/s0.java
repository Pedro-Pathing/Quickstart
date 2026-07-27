package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.C0285j;
import com.android.tools.r8.graph.C0286j0;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.I2;
import com.android.tools.r8.internal.AbstractC3250vj0;
import com.android.tools.r8.internal.AbstractC3424xb0;
import com.android.tools.r8.internal.C0521Db;
import com.android.tools.r8.internal.C0553Eb;
import com.android.tools.r8.internal.C0874Ot;
import com.android.tools.r8.internal.C1494d40;
import com.android.tools.r8.internal.C2317la0;
import com.android.tools.r8.internal.C2409ma0;
import com.android.tools.r8.internal.C2947sS;
import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.internal.EnumC3471y2;
import com.android.tools.r8.internal.Fh0;
import com.android.tools.r8.internal.H5;
import com.android.tools.r8.internal.InterfaceC1160Yw;
import com.android.tools.r8.internal.Vm0;
import com.android.tools.r8.synthesis.S;
import java.util.Collections;
import java.util.Set;
import java.util.function.Consumer;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class s0 {
    public static final /* synthetic */ boolean d = true;
    public final C0421y a;
    public final com.android.tools.r8.androidapi.a b;
    public final com.android.tools.r8.synthesis.J c;

    public s0(C0421y c0421y) {
        this.a = c0421y;
        this.b = c0421y.T;
        this.c = c0421y.a.g();
    }

    public final void a(A5 a5, C0874Ot c0874Ot, Fh0 fh0) {
        fh0.a("Compute and insert checkcast on return values");
        if (!a(this.a.h(), a5, c0874Ot).isEmpty()) {
            InterfaceC1160Yw interfaceC1160YwR = c0874Ot.r();
            while (interfaceC1160YwR.hasNext()) {
                C2409ma0 c2409ma0E0 = interfaceC1160YwR.next().E0();
                if (c2409ma0E0 != null) {
                    I2 i2G = a5.G();
                    C3161ul0 c3161ul0M2 = c2409ma0E0.M2();
                    boolean z = C0553Eb.k;
                    C0521Db c0521Db = new C0521Db();
                    c0521Db.e = c3161ul0M2;
                    C0421y c0421y = this.a;
                    C2947sS c2947sSN = c3161ul0M2.t().N();
                    i2G.getClass();
                    c0521Db.a = c0874Ot.a(AbstractC3250vj0.a(i2G, c2947sSN, (C0421y<?>) c0421y), (C0286j0) null);
                    C0521Db c0521Db2 = (C0521Db) c0521Db.a();
                    c0521Db2.d = i2G;
                    c0521Db2.b = c2409ma0E0.getPosition();
                    C0553Eb c0553EbC = c0521Db2.c();
                    interfaceC1160YwR.e(c0553EbC);
                    C2317la0 c2317la0 = new C2317la0();
                    c2317la0.b = c2409ma0E0.getPosition();
                    C3161ul0 c3161ul0D = c0553EbC.d();
                    c2317la0.d = c3161ul0D;
                    interfaceC1160YwR.add((C2409ma0) c2317la0.a(c3161ul0D == null ? new C2409ma0() : new C2409ma0(c2317la0.d)));
                }
            }
        }
        fh0.b();
    }

    public final Set a(final C0285j c0285j, A5 a5, C0874Ot c0874Ot) {
        if (!this.c.a(a5.s(), new com.android.tools.r8.synthesis.I() { // from class: com.android.tools.r8.ir.optimize.s0$$ExternalSyntheticLambda0
            @Override // com.android.tools.r8.synthesis.I
            public final S.b a(com.android.tools.r8.synthesis.S s) {
                return s.P;
            }
        }) && !this.c.a(a5.s(), new com.android.tools.r8.synthesis.I() { // from class: com.android.tools.r8.ir.optimize.s0$$ExternalSyntheticLambda1
            @Override // com.android.tools.r8.synthesis.I
            public final S.b a(com.android.tools.r8.synthesis.S s) {
                return s.Q;
            }
        })) {
            final I2 i2G = a5.G();
            if (!i2G.M0()) {
                return Collections.emptySet();
            }
            if (i2G == this.a.a().a2) {
                return Collections.emptySet();
            }
            com.android.tools.r8.graph.E0 e0D = c0285j.d(i2G);
            if (e0D != null && e0D.b0()) {
                com.android.tools.r8.androidapi.a aVar = this.b;
                int i = com.android.tools.r8.androidapi.f.a;
                if (aVar.a(i2G, com.android.tools.r8.androidapi.h.b).E()) {
                    return Collections.emptySet();
                }
                final Set setC = AbstractC3424xb0.c();
                final Set setC2 = AbstractC3424xb0.c();
                c0874Ot.e().forEach(new Consumer() { // from class: com.android.tools.r8.ir.optimize.s0$$ExternalSyntheticLambda2
                    @Override // java.util.function.Consumer
                    public final void accept(Object obj) {
                        this.f$0.a(c0285j, i2G, setC, setC2, (H5) obj);
                    }
                });
                return setC2;
            }
            return Collections.emptySet();
        }
        return Collections.emptySet();
    }

    public final void a(C0285j c0285j, I2 i2, Set set, Set set2, H5 h5) {
        boolean z;
        C2409ma0 c2409ma0E0 = h5.h().E0();
        C3161ul0 c3161ul0I = c2409ma0E0.M2().i();
        Vm0 vm0 = new Vm0(set);
        vm0.b(c3161ul0I);
        while (true) {
            if (!vm0.b()) {
                z = false;
                break;
            }
            C3161ul0 c3161ul0 = (C3161ul0) vm0.d();
            if (c3161ul0.l()) {
                vm0.b((Iterable) c3161ul0.p().c0());
            }
            AbstractC3250vj0 abstractC3250vj0T = c3161ul0.t();
            if (!abstractC3250vj0T.w()) {
                if (!d && !(abstractC3250vj0T instanceof C1494d40) && !abstractC3250vj0T.r()) {
                    throw new AssertionError();
                }
            } else {
                I2 i2Q = abstractC3250vj0T.b().Q();
                com.android.tools.r8.graph.E0 e0D = c0285j.d(i2Q);
                if (e0D != null && e0D.b0() && c0285j.b(i2Q, i2)) {
                    com.android.tools.r8.androidapi.a aVar = this.b;
                    int i = com.android.tools.r8.androidapi.f.a;
                    com.android.tools.r8.androidapi.f fVarA = aVar.a(i2Q, com.android.tools.r8.androidapi.h.b);
                    com.android.tools.r8.androidapi.f fVar = this.a.U;
                    if (!fVarA.E() && (!fVarA.v() || !fVarA.Y().a().b(EnumC3471y2.k))) {
                        if (fVarA.a(fVar)) {
                            z = true;
                            if (fVar.a(EnumC3471y2.v).c() ? true : fVarA.b(EnumC3471y2.F).c()) {
                                break;
                            }
                        } else {
                            continue;
                        }
                    }
                }
            }
        }
        if (z) {
            set2.add(c2409ma0E0);
        }
    }
}
