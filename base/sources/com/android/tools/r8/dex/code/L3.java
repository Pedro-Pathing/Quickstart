package com.android.tools.r8.dex.code;

import com.android.tools.r8.internal.C0756Kt;
import com.android.tools.r8.internal.C2108ja0;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes.dex */
public class L3 extends X0 {
    public L3(int i, B1 b1) {
        super(i, b1);
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final boolean C() {
        return true;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final void a(C0756Kt c0756Kt) {
        int iN = n();
        int i = this.g + iN;
        short s = this.f;
        c0756Kt.u.a(s, iN + 3, i, c0756Kt);
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String m() {
        return "SparseSwitch";
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final int p() {
        return 44;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String u() {
        return "sparse-switch";
    }

    public L3(int i) {
        super(i);
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String a(C2108ja0 c2108ja0) {
        return a("v" + ((int) this.f) + ", :label_" + (n() + this.g));
    }
}
