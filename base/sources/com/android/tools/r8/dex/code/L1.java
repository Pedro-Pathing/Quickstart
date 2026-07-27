package com.android.tools.r8.dex.code;

import com.android.tools.r8.graph.C0365q5;
import com.android.tools.r8.graph.C0409w2;
import com.android.tools.r8.graph.Y5;
import com.android.tools.r8.internal.C0756Kt;
import com.android.tools.r8.internal.EnumC2741qC;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes.dex */
public class L1 extends P1 {
    public L1(int i, B1 b1, C0365q5 c0365q5) {
        super(i, b1, c0365q5.b());
    }

    @Override // com.android.tools.r8.dex.code.P1
    public final EnumC2741qC I() {
        return EnumC2741qC.d;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final void a(Y5 y5) {
        y5.a(l());
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final boolean h() {
        return true;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String m() {
        return "InvokeDirect";
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final int p() {
        return 112;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String u() {
        return "invoke-direct";
    }

    public L1(int i, C0409w2 c0409w2, int i2, int i3, int i4, int i5, int i6) {
        super(i, c0409w2, i2, i3, i4, i5, i6);
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final void a(C0756Kt c0756Kt) {
        c0756Kt.a(EnumC2741qC.d, l(), (com.android.tools.r8.graph.E2) null, this.f, new int[]{this.g, this.h, this.i, this.j, this.k});
    }
}
