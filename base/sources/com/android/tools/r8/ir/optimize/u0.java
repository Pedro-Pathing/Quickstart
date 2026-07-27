package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.C0409w2;
import com.android.tools.r8.internal.AbstractC0695Iu;
import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.AbstractC3250vj0;
import com.android.tools.r8.internal.C0874Ot;
import com.android.tools.r8.internal.C2384mC;
import com.android.tools.r8.internal.C2929sC;
import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.internal.E6;
import com.android.tools.r8.internal.InterfaceC1160Yw;
import com.android.tools.r8.internal.S40;
import java.util.function.Consumer;
import java.util.function.Predicate;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class u0 {
    public final C0874Ot a;
    public final C2384mC b;
    public final InterfaceC1160Yw c;

    public u0(C0874Ot c0874Ot, InterfaceC1160Yw interfaceC1160Yw, C2384mC c2384mC) {
        this.c = interfaceC1160Yw;
        this.a = c0874Ot;
        this.b = c2384mC;
    }

    public final void a(C2929sC c2929sC, C0409w2 c0409w2) {
        if (c2929sC != null) {
            final E6 e6 = new E6(!c2929sC.d().A());
            c2929sC.d().c().forEach(new Consumer() { // from class: com.android.tools.r8.ir.optimize.u0$$ExternalSyntheticLambda0
                @Override // java.util.function.Consumer
                public final void accept(Object obj) {
                    this.f$0.a(e6, (AbstractC1076Vw) obj);
                }
            });
            if (e6.a()) {
                while (true) {
                    if (!this.c.hasPrevious()) {
                        break;
                    }
                    if (this.c.previous() == c2929sC) {
                        InterfaceC1160Yw interfaceC1160Yw = this.c;
                        C0874Ot c0874Ot = this.a;
                        c0874Ot.getClass();
                        interfaceC1160Yw.e(c0874Ot.a(0L, AbstractC3250vj0.m()));
                        break;
                    }
                }
                this.c.a(new Predicate() { // from class: com.android.tools.r8.ir.optimize.u0$$ExternalSyntheticLambda1
                    @Override // java.util.function.Predicate
                    public final boolean test(Object obj) {
                        return this.f$0.a((AbstractC1076Vw) obj);
                    }
                });
            }
        }
        final C2929sC c2929sCI0 = this.b.d().Z().i0();
        InterfaceC1160Yw interfaceC1160Yw2 = this.c;
        C0874Ot c0874Ot2 = this.a;
        c0874Ot2.getClass();
        interfaceC1160Yw2.e(c0874Ot2.a(0L, AbstractC3250vj0.m()));
        this.c.a(new Predicate() { // from class: com.android.tools.r8.ir.optimize.u0$$ExternalSyntheticLambda2
            @Override // java.util.function.Predicate
            public final boolean test(Object obj) {
                return u0.a(c2929sCI0, (AbstractC1076Vw) obj);
            }
        });
        C3161ul0 c3161ul0D = c2929sCI0.d();
        int i = AbstractC0695Iu.c;
        this.c.e(new C2384mC(c0409w2, c3161ul0D, S40.e));
    }

    public final void a(E6 e6, AbstractC1076Vw abstractC1076Vw) {
        e6.b(abstractC1076Vw == this.b && e6.a);
    }

    public final /* synthetic */ boolean a(AbstractC1076Vw abstractC1076Vw) {
        return abstractC1076Vw == this.b;
    }

    public static /* synthetic */ boolean a(C2929sC c2929sC, AbstractC1076Vw abstractC1076Vw) {
        return abstractC1076Vw == c2929sC;
    }
}
