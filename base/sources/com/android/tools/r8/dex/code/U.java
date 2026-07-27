package com.android.tools.r8.dex.code;

import com.android.tools.r8.internal.C0756Kt;
import com.android.tools.r8.internal.EnumC0555Ed;
import com.android.tools.r8.internal.WS;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes.dex */
public final class U extends T0 {
    public U(int i, B1 b1) {
        super(i, b1);
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final void a(C0756Kt c0756Kt) {
        c0756Kt.a(WS.h, EnumC0555Ed.d, this.f, this.g, this.h);
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String m() {
        return "CmplDouble";
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final int p() {
        return 47;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String u() {
        return "cmpl-double";
    }

    public U(int i, int i2, int i3) {
        super(i, i2, i3);
    }
}
