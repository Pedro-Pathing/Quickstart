package com.android.tools.r8.ir.optimize.info;

import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.AbstractC0324n0;
import com.android.tools.r8.graph.C0409w2;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.S4;
import com.android.tools.r8.internal.AbstractC3650zs;
import com.android.tools.r8.internal.EnumC2741qC;
import com.android.tools.r8.shaking.C3877i;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class t extends AbstractC0324n0 {
    public final C0421y f;

    public t(C0421y c0421y, A5 a5) {
        super(c0421y, a5);
        this.f = c0421y;
    }

    @Override // com.android.tools.r8.graph.AbstractC0324n0, com.android.tools.r8.graph.Y5
    public final void a(C0409w2 c0409w2) {
        AbstractC3650zs abstractC3650zsA = this.f.A();
        A5 a5 = (A5) this.b;
        abstractC3650zsA.getClass();
        i((C0409w2) abstractC3650zsA.a(c0409w2, a5.getReference(), EnumC2741qC.d, (AbstractC3650zs) null).a);
    }

    @Override // com.android.tools.r8.graph.AbstractC0324n0, com.android.tools.r8.graph.Y5
    public final void b(C0409w2 c0409w2) {
        AbstractC3650zs abstractC3650zsA = this.f.A();
        A5 a5 = (A5) this.b;
        abstractC3650zsA.getClass();
        i((C0409w2) abstractC3650zsA.a(c0409w2, a5.getReference(), EnumC2741qC.e, (AbstractC3650zs) null).a);
    }

    @Override // com.android.tools.r8.graph.AbstractC0324n0, com.android.tools.r8.graph.Y5
    public final void e(C0409w2 c0409w2) {
        AbstractC3650zs abstractC3650zsA = this.f.A();
        A5 a5 = (A5) this.b;
        abstractC3650zsA.getClass();
        i((C0409w2) abstractC3650zsA.a(c0409w2, a5.getReference(), EnumC2741qC.f, (AbstractC3650zs) null).a);
    }

    @Override // com.android.tools.r8.graph.AbstractC0324n0, com.android.tools.r8.graph.Y5
    public final void g(C0409w2 c0409w2) {
        AbstractC3650zs abstractC3650zsA = this.f.A();
        A5 a5 = (A5) this.b;
        abstractC3650zsA.getClass();
        i((C0409w2) abstractC3650zsA.a(c0409w2, a5.getReference(), EnumC2741qC.g, (AbstractC3650zs) null).a);
    }

    @Override // com.android.tools.r8.graph.AbstractC0324n0, com.android.tools.r8.graph.Y5
    public final void h(C0409w2 c0409w2) {
        AbstractC3650zs abstractC3650zsA = this.f.A();
        A5 a5 = (A5) this.b;
        abstractC3650zsA.getClass();
        i((C0409w2) abstractC3650zsA.a(c0409w2, a5.getReference(), EnumC2741qC.h, (AbstractC3650zs) null).a);
    }

    public final void i(C0409w2 c0409w2) {
        S4.c<?> cVarO = ((C3877i) this.f.g()).f(c0409w2).o();
        if (cVarO == null) {
            return;
        }
        h hVarA = this.f.o.a(cVarO.d(), cVarO.q());
        hVarA.getClass();
        if (hVarA instanceof C3682d) {
            return;
        }
        this.e = Boolean.TRUE;
        a();
    }
}
