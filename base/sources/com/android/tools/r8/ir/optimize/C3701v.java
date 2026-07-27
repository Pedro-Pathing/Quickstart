package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.internal.Y1;
import java.util.IdentityHashMap;

/* JADX INFO: renamed from: com.android.tools.r8.ir.optimize.v, reason: case insensitive filesystem */
/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class C3701v implements Y1 {
    public final IdentityHashMap b;

    public C3701v(IdentityHashMap identityHashMap) {
        IdentityHashMap identityHashMap2 = new IdentityHashMap();
        this.b = identityHashMap2;
        identityHashMap2.putAll(identityHashMap);
    }

    public final synchronized void a(IdentityHashMap identityHashMap) {
        this.b.putAll(identityHashMap);
    }

    @Override // com.android.tools.r8.internal.Y1
    public final void b() {
        this.b.forEach(new v$$ExternalSyntheticLambda0());
    }
}
