package com.android.tools.r8.ir.optimize.info;

import com.android.tools.r8.graph.B2;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.internal.C0918Ql;
import java.util.Collections;
import java.util.function.BiConsumer;
import java.util.function.BiFunction;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class p extends o {
    public final C0918Ql a;
    public static final /* synthetic */ boolean c = true;
    public static final p b = new p(new C0918Ql(Collections.emptyMap()));

    public p(C0918Ql c0918Ql) {
        this.a = c0918Ql;
    }

    /* JADX WARN: Removed duplicated region for block: B:52:0x00f0  */
    /* JADX WARN: Removed duplicated region for block: B:69:0x0123  */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public static com.android.tools.r8.ir.optimize.info.h a(com.android.tools.r8.ir.optimize.info.h r9, com.android.tools.r8.graph.C0421y r10, com.android.tools.r8.graph.B2 r11, com.android.tools.r8.ir.optimize.info.h r12) {
        /*
            Method dump skipped, instruction units count: 400
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.android.tools.r8.ir.optimize.info.p.a(com.android.tools.r8.ir.optimize.info.h, com.android.tools.r8.graph.y, com.android.tools.r8.graph.B2, com.android.tools.r8.ir.optimize.info.h):com.android.tools.r8.ir.optimize.info.h");
    }

    @Override // com.android.tools.r8.ir.optimize.info.o
    public final p b() {
        return this;
    }

    public final void a(final C0421y c0421y, p pVar) {
        pVar.a.forEach(new BiConsumer() { // from class: com.android.tools.r8.ir.optimize.info.p$$ExternalSyntheticLambda0
            @Override // java.util.function.BiConsumer
            public final void accept(Object obj, Object obj2) {
                this.f$0.b(c0421y, (B2) obj, (h) obj2);
            }
        });
    }

    /* JADX INFO: renamed from: a, reason: merged with bridge method [inline-methods] */
    public final void b(final C0421y c0421y, B2 b2, final h hVar) {
        this.a.a(b2, new BiFunction() { // from class: com.android.tools.r8.ir.optimize.info.p$$ExternalSyntheticLambda1
            @Override // java.util.function.BiFunction
            public final Object apply(Object obj, Object obj2) {
                return p.a(hVar, c0421y, (B2) obj, (h) obj2);
            }
        });
    }
}
