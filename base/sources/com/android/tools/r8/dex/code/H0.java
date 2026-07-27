package com.android.tools.r8.dex.code;

import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.C0357p5;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.internal.AbstractC3650zs;
import com.android.tools.r8.internal.C2108ja0;
import com.android.tools.r8.internal.SJ;
import com.android.tools.r8.utils.structural.AbstractC4012a;
import java.nio.ShortBuffer;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes.dex */
public abstract class H0 extends J {
    public static final /* synthetic */ boolean g = true;
    public final short f;

    public H0(int i, B1 b1) {
        super(b1);
        this.f = (short) i;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final void a(C0357p5 c0357p5, A5 a5, AbstractC3650zs abstractC3650zs, AbstractC3650zs abstractC3650zs2, SJ sj, ShortBuffer shortBuffer) {
        A1.a(this.f, p(), shortBuffer);
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final void b(com.android.tools.r8.utils.structural.o oVar) {
        ((com.android.tools.r8.utils.structural.q) oVar).a.a((int) this.f);
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final int hashCode() {
        return this.f ^ getClass().hashCode();
    }

    public H0(int i) {
        if (!g && (i < 0 || i > 255)) {
            throw new AssertionError();
        }
        this.f = (short) i;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final int a(A1 a1, AbstractC4012a abstractC4012a) {
        return abstractC4012a.a((int) this.f, (int) ((H0) a1).f);
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String a(C2108ja0 c2108ja0) {
        return a("v" + ((int) this.f));
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String b(C2108ja0 c2108ja0) {
        return b("v" + ((int) this.f));
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final void a(C0421y c0421y, AbstractC3650zs abstractC3650zs, com.android.tools.r8.dex.M m, A5 a5, SJ sj) {
    }
}
