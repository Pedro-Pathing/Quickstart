package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.internal.AbstractC3424xb0;
import com.android.tools.r8.internal.C0874Ot;
import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.internal.C3571z4;
import com.android.tools.r8.internal.K5;
import java.util.Set;
import java.util.function.Consumer;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class r {
    public static final /* synthetic */ boolean d = true;
    public final C0421y a;
    public final C0874Ot b;
    public final Set c = AbstractC3424xb0.c();

    public r(C0421y c0421y, C0874Ot c0874Ot) {
        this.a = c0421y;
        this.b = c0874Ot;
    }

    public final void a(C3571z4 c3571z4) {
        this.c.add(c3571z4);
    }

    public static void a(C3571z4 c3571z4, K5 k5, Set set, Consumer consumer) {
        C3161ul0 c3161ul0L2 = c3571z4.L2();
        C3161ul0 c3161ul0D = c3571z4.d();
        if (c3161ul0D == null) {
            return;
        }
        if (!c3161ul0D.t().equals(c3161ul0L2.t())) {
            set.addAll(c3161ul0D.a());
        }
        c3161ul0D.f(c3161ul0L2);
        consumer.accept(c3571z4);
        k5.p();
    }
}
