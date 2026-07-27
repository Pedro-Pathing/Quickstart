package com.android.tools.r8.ir.optimize.info;

import com.android.tools.r8.graph.AbstractC0264h1;
import com.android.tools.r8.graph.D2;
import com.android.tools.r8.internal.BO;
import com.android.tools.r8.internal.C0871Oq;
import com.android.tools.r8.internal.C1552dh0;
import com.android.tools.r8.internal.InterfaceC3551yp;
import com.android.tools.r8.threading.ThreadingModule;
import java.util.Collection;
import java.util.Objects;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.function.Consumer;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public abstract class y implements InterfaceC3551yp, BO {
    public static final /* synthetic */ boolean a = true;

    public void a(Consumer consumer) {
    }

    public static C a() {
        return C.b;
    }

    public static void a(Collection collection, ThreadingModule threadingModule, ExecutorService executorService, final x xVar) throws ExecutionException {
        C1552dh0.a(collection, new Consumer() { // from class: com.android.tools.r8.ir.optimize.info.y$$ExternalSyntheticLambda1
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                y.a(xVar, (D2) obj);
            }
        }, threadingModule, executorService);
    }

    public static /* synthetic */ void a(final x xVar, D2 d2) {
        C0871Oq c0871OqB1 = d2.B1();
        Objects.requireNonNull(xVar);
        c0871OqB1.forEach(new Consumer() { // from class: com.android.tools.r8.ir.optimize.info.y$$ExternalSyntheticLambda0
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                xVar.a((AbstractC0264h1) obj);
            }
        });
    }
}
