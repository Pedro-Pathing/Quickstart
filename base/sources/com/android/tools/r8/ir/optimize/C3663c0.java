package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.C0282i5;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.D2;
import com.android.tools.r8.graph.I2;
import com.android.tools.r8.internal.AbstractC0897Pq;
import com.android.tools.r8.internal.AbstractC3424xb0;
import com.android.tools.r8.internal.C1552dh0;
import com.android.tools.r8.internal.E6;
import com.android.tools.r8.internal.Fh0;
import com.android.tools.r8.internal.LC;
import com.android.tools.r8.shaking.C3877i;
import com.android.tools.r8.threading.ThreadingModule;
import java.util.Set;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Predicate;

/* JADX INFO: renamed from: com.android.tools.r8.ir.optimize.c0, reason: case insensitive filesystem */
/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class C3663c0 {
    public final C0421y a;

    public C3663c0(C0421y c0421y) {
        this.a = c0421y;
    }

    public final void a(ExecutorService executorService, Fh0 fh0) throws ExecutionException {
        fh0.a("NestReduction");
        if (this.a.M().l()) {
            a(executorService);
        } else {
            for (D2 d2 : ((C3877i) this.a.g()).d()) {
                if (d2.t1()) {
                    if (d2.w1()) {
                        d2.p.clear();
                    } else {
                        d2.o = null;
                    }
                }
            }
        }
        this.a.getClass();
        fh0.b();
    }

    public final void b(final D2 d2) {
        c0$$ExternalSyntheticLambda1 c0__externalsyntheticlambda1 = new c0$$ExternalSyntheticLambda1();
        final E6 e6 = new E6(((LC) AbstractC0897Pq.a(d2.c(c0__externalsyntheticlambda1), d2.f(c0__externalsyntheticlambda1)).iterator()).hasNext());
        d2.Y0().removeIf(new Predicate() { // from class: com.android.tools.r8.ir.optimize.c0$$ExternalSyntheticLambda4
            @Override // java.util.function.Predicate
            public final boolean test(Object obj) {
                return this.f$0.a(d2, e6, (C0282i5) obj);
            }
        });
        if (e6.d() && this.a.M().H0) {
            d2.Y0().clear();
        }
    }

    public final void c(D2 d2) {
        C0421y c0421y = this.a;
        I2 i2W0 = d2.W0();
        c0421y.getClass();
        D2 d2B = D2.b(i2W0 == d2.e ? d2 : c0421y.a(i2W0));
        if (d2B == null || !d2B.w1()) {
            d2.o = null;
        }
    }

    public final void a(ExecutorService executorService) throws ExecutionException {
        Set setC = AbstractC3424xb0.c();
        Set setC2 = AbstractC3424xb0.c();
        for (D2 d2 : ((C3877i) this.a.g()).d()) {
            if (d2.t1()) {
                if (d2.w1()) {
                    setC.add(d2);
                } else {
                    setC2.add(d2);
                }
            }
        }
        ThreadingModule threadingModuleN = this.a.M().N();
        C1552dh0.a(setC, new Consumer() { // from class: com.android.tools.r8.ir.optimize.c0$$ExternalSyntheticLambda2
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                this.f$0.b((D2) obj);
            }
        }, threadingModuleN, executorService);
        C1552dh0.a(setC2, new Consumer() { // from class: com.android.tools.r8.ir.optimize.c0$$ExternalSyntheticLambda3
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                this.f$0.c((D2) obj);
            }
        }, threadingModuleN, executorService);
    }

    public final /* synthetic */ boolean a(D2 d2, E6 e6, C0282i5 c0282i5) {
        final D2 d2B = D2.b(this.a.a(d2, c0282i5.a()));
        if (d2B == null) {
            return true;
        }
        e6.a(new BooleanSupplier() { // from class: com.android.tools.r8.ir.optimize.c0$$ExternalSyntheticLambda0
            @Override // java.util.function.BooleanSupplier
            public final boolean getAsBoolean() {
                return C3663c0.a(d2B);
            }
        });
        return false;
    }

    public static boolean a(D2 d2) {
        c0$$ExternalSyntheticLambda1 c0__externalsyntheticlambda1 = new c0$$ExternalSyntheticLambda1();
        return ((LC) AbstractC0897Pq.a(d2.c(c0__externalsyntheticlambda1), d2.f(c0__externalsyntheticlambda1)).iterator()).hasNext();
    }
}
