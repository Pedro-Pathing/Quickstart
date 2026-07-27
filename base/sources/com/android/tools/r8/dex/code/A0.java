package com.android.tools.r8.dex.code;

import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.AbstractC0325n1;
import com.android.tools.r8.graph.C0357p5;
import com.android.tools.r8.graph.C0365q5;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.internal.AbstractC3650zs;
import com.android.tools.r8.internal.C0756Kt;
import com.android.tools.r8.internal.EnumC2741qC;
import com.android.tools.r8.internal.Jl0;
import com.android.tools.r8.internal.SJ;
import java.nio.ShortBuffer;
import java.util.ArrayList;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes.dex */
public class A0 extends AbstractC0039a1 {
    public A0(int i, B1 b1, C0365q5 c0365q5) {
        super(i, b1, c0365q5.c());
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final void a(C0421y c0421y, AbstractC3650zs abstractC3650zs, com.android.tools.r8.dex.M m, A5 a5, SJ sj) {
        c0421y.A().c(abstractC3650zs, (com.android.tools.r8.graph.I2) this.h).a(c0421y, m);
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final boolean h() {
        return true;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String m() {
        return "FilledNewArrayRange";
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final int p() {
        return 37;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String u() {
        return "filled-new-array/range";
    }

    public A0(int i, int i2, com.android.tools.r8.graph.I2 i22) {
        super(i, i2, i22);
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final void a(C0756Kt c0756Kt) {
        int i;
        com.android.tools.r8.graph.I2 i2 = (com.android.tools.r8.graph.I2) this.h;
        int i3 = this.f;
        int i4 = this.g;
        c0756Kt.getClass();
        String string = i2.f.toString();
        boolean z = C0756Kt.D;
        if (!z && string.charAt(0) != '[') {
            throw new AssertionError();
        }
        if (!z && string.length() < 2) {
            throw new AssertionError();
        }
        Jl0 jl0A = Jl0.a(string.charAt(1));
        ArrayList arrayList = new ArrayList(i3 / jl0A.c());
        int iC = i4;
        while (true) {
            i = i4 + i3;
            if (iC >= i) {
                break;
            }
            arrayList.add(c0756Kt.b(iC, jl0A));
            iC += jl0A.c();
        }
        C0756Kt.a(iC, i);
        if (!C0756Kt.D && !c0756Kt.p.M().Z()) {
            throw new AssertionError();
        }
        c0756Kt.a(EnumC2741qC.i, (AbstractC0325n1) i2, (com.android.tools.r8.graph.E2) null, arrayList, false);
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final void a(C0357p5 c0357p5, A5 a5, AbstractC3650zs abstractC3650zs, AbstractC3650zs abstractC3650zs2, SJ sj, ShortBuffer shortBuffer) {
        com.android.tools.r8.graph.I2 i2C = abstractC3650zs.c(abstractC3650zs2, (com.android.tools.r8.graph.I2) this.h);
        A1.a(this.f, 37, shortBuffer);
        A1.a(i2C, shortBuffer, c0357p5);
        shortBuffer.put((short) this.g);
    }
}
