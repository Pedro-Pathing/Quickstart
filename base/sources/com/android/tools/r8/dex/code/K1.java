package com.android.tools.r8.dex.code;

import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.C0357p5;
import com.android.tools.r8.graph.C0365q5;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.Y5;
import com.android.tools.r8.internal.AbstractC3650zs;
import com.android.tools.r8.internal.SJ;
import java.nio.ShortBuffer;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes.dex */
public class K1 extends AbstractC0039a1 {
    public K1(int i, int i2, com.android.tools.r8.graph.D0 d0) {
        super(i, i2, d0);
    }

    /* JADX WARN: Removed duplicated region for block: B:10:0x002e  */
    @Override // com.android.tools.r8.dex.code.A1
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public final void a(com.android.tools.r8.internal.C0756Kt r10) {
        /*
            r9 = this;
            com.android.tools.r8.graph.W3 r0 = r9.h
            com.android.tools.r8.graph.D0 r0 = (com.android.tools.r8.graph.D0) r0
            short r1 = r9.f
            char r2 = r9.g
            r10.getClass()
            com.android.tools.r8.graph.y2 r3 = r0.g
            java.util.ArrayList r4 = new java.util.ArrayList
            r4.<init>(r1)
            com.android.tools.r8.graph.x2 r5 = r3.e
            r5.getClass()
            com.android.tools.r8.graph.x2 r6 = com.android.tools.r8.graph.EnumC0417x2.c
            if (r5 != r6) goto L1c
            goto L2e
        L1c:
            com.android.tools.r8.graph.x2 r5 = r3.e
            r5.getClass()
            com.android.tools.r8.graph.x2 r6 = com.android.tools.r8.graph.EnumC0417x2.d
            if (r5 != r6) goto L26
            goto L2e
        L26:
            com.android.tools.r8.graph.x2 r3 = r3.e
            boolean r3 = r3.f()
            if (r3 == 0) goto L30
        L2e:
            r3 = r2
            goto L3e
        L30:
            com.android.tools.r8.internal.Jl0 r3 = com.android.tools.r8.internal.Jl0.b
            com.android.tools.r8.internal.ul0 r5 = r10.b(r2, r3)
            r4.add(r5)
            int r3 = r3.c()
            int r3 = r3 + r2
        L3e:
            com.android.tools.r8.graph.E2 r5 = r0.f
            java.lang.String r5 = r5.o0()
            r6 = 1
        L45:
            int r7 = r5.length()
            if (r6 >= r7) goto L62
            char r7 = r5.charAt(r6)
            com.android.tools.r8.internal.Jl0 r7 = com.android.tools.r8.internal.Jl0.a(r7)
            com.android.tools.r8.internal.ul0 r8 = r10.b(r3, r7)
            r4.add(r8)
            int r7 = r7.c()
            int r3 = r3 + r7
            int r6 = r6 + 1
            goto L45
        L62:
            int r2 = r2 + r1
            com.android.tools.r8.internal.C0756Kt.a(r3, r2)
            com.android.tools.r8.internal.UB r1 = new com.android.tools.r8.internal.UB
            r2 = 0
            r1.<init>(r0, r2, r4)
            r10.a(r1)
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: com.android.tools.r8.dex.code.K1.a(com.android.tools.r8.internal.Kt):void");
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final boolean h() {
        return true;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final com.android.tools.r8.graph.D0 j() {
        return (com.android.tools.r8.graph.D0) this.h;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String m() {
        return "InvokeCustomRange";
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final int p() {
        return 253;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String u() {
        return "invoke-custom/range";
    }

    /* JADX WARN: 'super' call moved to the top of the method (can break code semantics) */
    public K1(int i, B1 b1, C0365q5 c0365q5) {
        super(i, b1, c0365q5.g);
        if (!C0365q5.i && c0365q5.g == null) {
            throw new AssertionError();
        }
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final void a(C0421y c0421y, AbstractC3650zs abstractC3650zs, com.android.tools.r8.dex.M m, A5 a5, SJ sj) {
        sj.a((com.android.tools.r8.graph.D0) this.h, a5).a(c0421y, m);
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final void a(Y5 y5) {
        y5.a((com.android.tools.r8.graph.D0) this.h);
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final void a(C0357p5 c0357p5, A5 a5, AbstractC3650zs abstractC3650zs, AbstractC3650zs abstractC3650zs2, SJ sj, ShortBuffer shortBuffer) {
        com.android.tools.r8.graph.D0 d0A = sj.a((com.android.tools.r8.graph.D0) this.h, a5);
        A1.a(this.f, 253, shortBuffer);
        A1.a(d0A, shortBuffer, c0357p5);
        shortBuffer.put((short) this.g);
    }
}
