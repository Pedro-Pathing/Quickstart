package com.android.tools.r8.dex.code;

import com.android.tools.r8.internal.C0756Kt;
import com.android.tools.r8.internal.WS;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes.dex */
public class k4 extends T0 {
    public k4(int i, B1 b1) {
        super(i, b1);
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final void a(C0756Kt c0756Kt) {
        c0756Kt.r(WS.e, this.f, this.g, this.h);
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String m() {
        return "XorInt";
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final int p() {
        return 151;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String u() {
        return "xor-int";
    }

    public k4(int i, int i2, int i3) {
        super(i, i2, i3);
    }
}
