package com.android.tools.r8.dex.code;

import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.C0357p5;
import com.android.tools.r8.graph.C0365q5;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.Y5;
import com.android.tools.r8.internal.AbstractC3650zs;
import com.android.tools.r8.internal.C0756Kt;
import com.android.tools.r8.internal.SJ;
import java.nio.ShortBuffer;
import java.util.function.Function;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes.dex */
public class P extends K0<com.android.tools.r8.graph.I2> {
    public final boolean i;

    public P(int i, B1 b1, C0365q5 c0365q5) {
        super(i, b1, c0365q5.c());
        this.i = false;
    }

    public com.android.tools.r8.graph.I2 I() {
        return (com.android.tools.r8.graph.I2) this.g;
    }

    @Override // com.android.tools.r8.dex.code.K0
    public final void a(com.android.tools.r8.utils.structural.A a) {
        a.e(new Function() { // from class: com.android.tools.r8.dex.code.P$$ExternalSyntheticLambda0
            @Override // java.util.function.Function
            public final Object apply(Object obj) {
                return P.b((K0) obj);
            }
        });
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final P b() {
        return this;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final boolean h() {
        return true;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String m() {
        return "CheckCast";
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final int p() {
        return 31;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String u() {
        return "check-cast";
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final boolean y() {
        return true;
    }

    public static /* synthetic */ com.android.tools.r8.graph.I2 b(K0 k0) {
        return (com.android.tools.r8.graph.I2) k0.g;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final void a(C0421y c0421y, AbstractC3650zs abstractC3650zs, com.android.tools.r8.dex.M m, A5 a5, SJ sj) {
        c0421y.A().c(abstractC3650zs, I()).a(c0421y, m);
    }

    public P(int i, com.android.tools.r8.graph.I2 i2, boolean z) {
        super(i, i2);
        this.i = z;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final void a(C0357p5 c0357p5, A5 a5, AbstractC3650zs abstractC3650zs, AbstractC3650zs abstractC3650zs2, SJ sj, ShortBuffer shortBuffer) {
        com.android.tools.r8.graph.I2 i2C = abstractC3650zs.c(abstractC3650zs2, I());
        A1.a(this.f, 31, shortBuffer);
        A1.a(i2C, shortBuffer, c0357p5);
    }

    @Override // com.android.tools.r8.dex.code.A1
    public void a(Y5 y5) {
        y5.a(I(), this.i);
    }

    @Override // com.android.tools.r8.dex.code.A1
    public void a(C0756Kt c0756Kt) {
        c0756Kt.a((int) this.f, I(), false);
    }
}
