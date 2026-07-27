package com.android.tools.r8.ir.optimize.info;

import com.android.tools.r8.graph.C0285j;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.V3;
import com.android.tools.r8.internal.BX;
import com.android.tools.r8.internal.C1552dh0;
import com.android.tools.r8.internal.MY;
import java.util.ArrayList;
import java.util.IdentityHashMap;
import java.util.Set;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.function.Consumer;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public abstract class q {
    public static void a(final C0421y c0421y, ExecutorService executorService, BX bx) throws ExecutionException {
        final V3 v3A = V3.a(c0421y, ((C0285j) c0421y.g()).d());
        ArrayList arrayListA = new MY(c0421y, v3A).a();
        s sVar = s.b;
        final r rVar = new r();
        C1552dh0.a(arrayListA, new Consumer() { // from class: com.android.tools.r8.ir.optimize.info.q$$ExternalSyntheticLambda0
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                new n(c0421y, rVar, v3A).a((Set) obj);
            }
        }, c0421y.M().N(), executorService);
        IdentityHashMap identityHashMap = new IdentityHashMap(rVar.a);
        c0421y.o = new s(identityHashMap);
        if (identityHashMap.isEmpty()) {
            return;
        }
        u uVar = new u(c0421y);
        bx.a(c0421y);
        uVar.a(bx, executorService);
    }
}
