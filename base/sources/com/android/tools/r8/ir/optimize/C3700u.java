package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.C0257g1;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.N2;
import java.util.IdentityHashMap;
import java.util.Map;
import java.util.function.BiConsumer;

/* JADX INFO: renamed from: com.android.tools.r8.ir.optimize.u, reason: case insensitive filesystem */
/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class C3700u {
    public final Map a;
    public static final /* synthetic */ boolean c = true;
    public static final C3700u b = new C3700u(null);

    public C3700u(IdentityHashMap identityHashMap) {
        this.a = identityHashMap;
    }

    public final void a(BiConsumer biConsumer) {
        Map map = this.a;
        if (map != null) {
            map.forEach(biConsumer);
        }
    }

    public final void a(final BiConsumer biConsumer, final C0421y c0421y) {
        a(new BiConsumer() { // from class: com.android.tools.r8.ir.optimize.u$$ExternalSyntheticLambda0
            @Override // java.util.function.BiConsumer
            public final void accept(Object obj, Object obj2) {
                N2 n2 = (N2) obj2;
                biConsumer.accept(((C0257g1) obj).a(c0421y), n2);
            }
        });
    }
}
