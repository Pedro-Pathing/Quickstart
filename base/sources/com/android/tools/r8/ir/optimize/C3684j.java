package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.AbstractC3250vj0;
import com.android.tools.r8.internal.AbstractC3424xb0;
import com.android.tools.r8.internal.C0621Gm;
import com.android.tools.r8.internal.C1494d40;
import com.android.tools.r8.internal.C3161ul0;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Function;

/* JADX INFO: renamed from: com.android.tools.r8.ir.optimize.j, reason: case insensitive filesystem */
/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class C3684j {
    public final LinkedHashMap a = new LinkedHashMap();
    public final Set b = AbstractC3424xb0.c();

    public final void a(AbstractC1076Vw abstractC1076Vw, C3161ul0 c3161ul0, final AbstractC3675h abstractC3675h, Consumer consumer) {
        C3677i c3677i = (C3677i) ((Map) this.a.computeIfAbsent(abstractC1076Vw, new Function() { // from class: com.android.tools.r8.ir.optimize.j$$ExternalSyntheticLambda2
            @Override // java.util.function.Function
            public final Object apply(Object obj) {
                return C3684j.a((AbstractC1076Vw) obj);
            }
        })).computeIfAbsent(c3161ul0, new Function() { // from class: com.android.tools.r8.ir.optimize.j$$ExternalSyntheticLambda3
            @Override // java.util.function.Function
            public final Object apply(Object obj) {
                return C3684j.a(abstractC3675h, (C3161ul0) obj);
            }
        });
        consumer.accept(c3677i);
        if ((abstractC3675h instanceof C3687l) && c3677i.b()) {
            this.b.add(c3161ul0);
        }
    }

    public final void b(AbstractC1076Vw abstractC1076Vw, C3161ul0 c3161ul0, final C0621Gm c0621Gm) {
        a(abstractC1076Vw, c3161ul0, C3691p.a, new Consumer() { // from class: com.android.tools.r8.ir.optimize.j$$ExternalSyntheticLambda0
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                ((C3677i) obj).a(c0621Gm);
            }
        });
    }

    public final void b(AbstractC1076Vw abstractC1076Vw, C3161ul0 c3161ul0) {
        a(abstractC1076Vw, c3161ul0, C3691p.a, new j$$ExternalSyntheticLambda4());
    }

    public static /* synthetic */ Map a(AbstractC1076Vw abstractC1076Vw) {
        return new LinkedHashMap();
    }

    public static /* synthetic */ C3677i a(AbstractC3675h abstractC3675h, C3161ul0 c3161ul0) {
        return new C3677i(abstractC3675h);
    }

    public final void a(AbstractC1076Vw abstractC1076Vw, C3161ul0 c3161ul0, final C0621Gm c0621Gm) {
        a(abstractC1076Vw, c3161ul0, C3687l.a, new Consumer() { // from class: com.android.tools.r8.ir.optimize.j$$ExternalSyntheticLambda1
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                ((C3677i) obj).a(c0621Gm);
            }
        });
    }

    public final void a(AbstractC1076Vw abstractC1076Vw, C3161ul0 c3161ul0) {
        a(abstractC1076Vw, c3161ul0, C3687l.a, new j$$ExternalSyntheticLambda4());
    }

    public final boolean a(C3161ul0 c3161ul0) {
        if (!this.b.contains(c3161ul0)) {
            AbstractC3250vj0 abstractC3250vj0T = c3161ul0.t();
            abstractC3250vj0T.getClass();
            if (!(abstractC3250vj0T instanceof C1494d40)) {
                return true;
            }
        }
        return false;
    }
}
