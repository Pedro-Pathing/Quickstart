package com.android.tools.r8.dex.code;

import com.android.tools.r8.internal.C0756Kt;
import com.android.tools.r8.internal.C1938hh0;
import com.android.tools.r8.internal.Jl0;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes.dex */
public class d4 extends H0 {
    public d4(int i, B1 b1) {
        super(i, b1);
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final void a(C0756Kt c0756Kt) {
        c0756Kt.a(c0756Kt.u.g(), new C1938hh0(c0756Kt.b(this.f, Jl0.b)));
        c0756Kt.c();
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final boolean h() {
        return true;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String m() {
        return "Throw";
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final int p() {
        return 39;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String u() {
        return "throw";
    }

    public d4(int i) {
        super(i);
    }
}
