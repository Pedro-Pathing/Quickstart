package com.android.tools.r8.ir.optimize.info;

import com.android.tools.r8.graph.C0291j1;
import com.android.tools.r8.graph.E0;
import java.util.Collections;
import java.util.Map;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class s {
    public static final s b = new s(Collections.emptyMap());
    public final Map a;

    public s(Map map) {
        this.a = map;
    }

    public final h a(E0 e0, C0291j1 c0291j1) {
        C3682d c3682d = C3682d.b;
        return !e0.a0() ? c3682d : (h) this.a.getOrDefault(c0291j1.getReference(), c3682d);
    }
}
