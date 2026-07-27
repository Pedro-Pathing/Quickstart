package com.android.tools.r8.ir.optimize.info;

import com.android.tools.r8.graph.AbstractC0264h1;
import com.android.tools.r8.graph.C0257g1;
import com.android.tools.r8.graph.C0291j1;
import java.util.function.Consumer;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public interface x {
    void a(C0257g1 c0257g1, v vVar);

    default void a(AbstractC0264h1 abstractC0264h1) {
        final g gVarH0 = abstractC0264h1.H0();
        if (gVarH0.d()) {
            abstractC0264h1.a(new Consumer() { // from class: com.android.tools.r8.ir.optimize.info.x$$ExternalSyntheticLambda0
                @Override // java.util.function.Consumer
                public final void accept(Object obj) {
                    this.f$0.a(gVarH0, (C0257g1) obj);
                }
            }, new Consumer() { // from class: com.android.tools.r8.ir.optimize.info.x$$ExternalSyntheticLambda1
                @Override // java.util.function.Consumer
                public final void accept(Object obj) {
                    this.f$0.a(gVarH0, (C0291j1) obj);
                }
            });
        }
    }

    void a(C0291j1 c0291j1, w wVar);

    /* synthetic */ default void a(g gVar, C0257g1 c0257g1) {
        a(c0257g1, gVar.c());
    }

    /* synthetic */ default void a(g gVar, C0291j1 c0291j1) {
        a(c0291j1, gVar.a());
    }
}
