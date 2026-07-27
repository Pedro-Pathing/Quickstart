package com.android.tools.r8.dex.code;

import com.android.tools.r8.internal.C0756Kt;
import com.android.tools.r8.internal.WS;
import com.sun.tools.javac.jvm.ByteCodes;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes.dex */
public class A extends T0 {
    public A(int i, B1 b1) {
        super(i, b1);
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final void a(C0756Kt c0756Kt) {
        c0756Kt.c(WS.f, this.f, this.g, this.h);
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String m() {
        return "AndLong";
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final int p() {
        return ByteCodes.if_icmpne;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String u() {
        return "and-long";
    }

    public A(int i, int i2, int i3) {
        super(i, i2, i3);
    }
}
