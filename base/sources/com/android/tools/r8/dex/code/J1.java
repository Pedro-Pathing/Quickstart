package com.android.tools.r8.dex.code;

import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.C0357p5;
import com.android.tools.r8.graph.C0365q5;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.C0424y2;
import com.android.tools.r8.graph.EnumC0417x2;
import com.android.tools.r8.graph.Y5;
import com.android.tools.r8.internal.AbstractC3650zs;
import com.android.tools.r8.internal.C0756Kt;
import com.android.tools.r8.internal.Jl0;
import com.android.tools.r8.internal.SJ;
import com.android.tools.r8.internal.UB;
import java.nio.ShortBuffer;
import java.util.ArrayList;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes.dex */
public class J1 extends Z0 {
    public J1(int i, com.android.tools.r8.graph.D0 d0, int i2, int i3, int i4, int i5, int i6) {
        super(i, d0, i2, i3, i4, i5, i6);
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final void a(C0756Kt c0756Kt) {
        com.android.tools.r8.graph.D0 d0 = (com.android.tools.r8.graph.D0) this.l;
        byte b = this.f;
        int[] iArr = {this.g, this.h, this.i, this.j, this.k};
        c0756Kt.getClass();
        C0424y2 c0424y2 = d0.g;
        ArrayList arrayList = new ArrayList(b);
        EnumC0417x2 enumC0417x2 = c0424y2.e;
        enumC0417x2.getClass();
        int iC = 0;
        if (enumC0417x2 != EnumC0417x2.c) {
            EnumC0417x2 enumC0417x22 = c0424y2.e;
            enumC0417x22.getClass();
            if (enumC0417x22 != EnumC0417x2.d && !c0424y2.e.f()) {
                int i = iArr[0];
                Jl0 jl0 = Jl0.b;
                arrayList.add(c0756Kt.b(i, jl0));
                iC = jl0.c();
            }
        }
        String strO0 = d0.f.o0();
        for (int i2 = 1; i2 < strO0.length(); i2++) {
            Jl0 jl0A = Jl0.a(strO0.charAt(i2));
            arrayList.add(c0756Kt.b(iArr[iC], jl0A));
            iC += jl0A.c();
        }
        c0756Kt.a(new UB(d0, null, arrayList));
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final boolean h() {
        return true;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final com.android.tools.r8.graph.D0 j() {
        return (com.android.tools.r8.graph.D0) this.l;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String m() {
        return "InvokeCustom";
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final int p() {
        return 252;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String u() {
        return "invoke-custom";
    }

    /* JADX WARN: 'super' call moved to the top of the method (can break code semantics) */
    public J1(int i, B1 b1, C0365q5 c0365q5) {
        super(i, b1, c0365q5.g);
        if (!C0365q5.i && c0365q5.g == null) {
            throw new AssertionError();
        }
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final void a(C0421y c0421y, AbstractC3650zs abstractC3650zs, com.android.tools.r8.dex.M m, A5 a5, SJ sj) {
        sj.a((com.android.tools.r8.graph.D0) this.l, a5).a(c0421y, m);
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final void a(Y5 y5) {
        y5.a((com.android.tools.r8.graph.D0) this.l);
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final void a(C0357p5 c0357p5, A5 a5, AbstractC3650zs abstractC3650zs, AbstractC3650zs abstractC3650zs2, SJ sj, ShortBuffer shortBuffer) {
        A1.a(this.f, this.k, shortBuffer, 252);
        A1.a(sj.a((com.android.tools.r8.graph.D0) this.l, a5), shortBuffer, c0357p5);
        shortBuffer.put(A1.d(A1.e(this.j, this.i), A1.e(this.h, this.g)));
    }
}
