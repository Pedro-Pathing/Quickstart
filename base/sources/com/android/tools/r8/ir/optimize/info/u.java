package com.android.tools.r8.ir.optimize.info;

import com.android.tools.r8.dex.k$$ExternalSyntheticLambda24;
import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.D2;
import com.android.tools.r8.internal.AbstractC3650zs;
import com.android.tools.r8.internal.BX;
import com.android.tools.r8.internal.C1552dh0;
import com.android.tools.r8.internal.C2298lM;
import com.android.tools.r8.internal.C3200vB;
import com.android.tools.r8.internal.InterfaceC2602oh0;
import com.android.tools.r8.shaking.C3877i;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.function.Consumer;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class u {
    public final C0421y a;

    public u(C0421y c0421y) {
        this.a = c0421y;
    }

    public final void a(final BX bx, ExecutorService executorService) {
        final AbstractC3650zs abstractC3650zsA = this.a.A();
        final C3200vB c3200vBM = this.a.M();
        C1552dh0.a(((C3877i) this.a.g()).d(), new InterfaceC2602oh0() { // from class: com.android.tools.r8.ir.optimize.info.u$$ExternalSyntheticLambda0
            @Override // com.android.tools.r8.internal.InterfaceC2602oh0
            public final Object apply(Object obj) {
                return this.f$0.a(bx, abstractC3650zsA, c3200vBM, (D2) obj);
            }
        }, c3200vBM.N(), executorService).forEach(new Consumer() { // from class: com.android.tools.r8.ir.optimize.info.u$$ExternalSyntheticLambda1
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                bx.a((List) obj, abstractC3650zsA);
            }
        });
    }

    public final /* synthetic */ List a(final BX bx, final AbstractC3650zs abstractC3650zs, final C3200vB c3200vB, D2 d2) {
        final ArrayList arrayList = new ArrayList();
        d2.h(new Consumer() { // from class: com.android.tools.r8.ir.optimize.info.u$$ExternalSyntheticLambda2
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                this.f$0.a(bx, abstractC3650zs, c3200vB, arrayList, (A5) obj);
            }
        }, new k$$ExternalSyntheticLambda24());
        return arrayList;
    }

    public final void a(BX bx, AbstractC3650zs abstractC3650zs, C3200vB c3200vB, List list, A5 a5) {
        C2298lM c2298lM = bx.a;
        c2298lM.getClass();
        boolean z = C2298lM.e;
        if (!z && !z && c2298lM.c != abstractC3650zs) {
            throw new AssertionError();
        }
        if (c2298lM.d.contains(a5.getReference()) || !this.a.a(a5).a(a5)) {
            return;
        }
        t tVar = new t(this.a, a5);
        a5.a(tVar);
        if (((Boolean) tVar.e).booleanValue()) {
            list.add(a5);
        }
    }
}
