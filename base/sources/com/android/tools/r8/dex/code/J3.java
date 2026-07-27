package com.android.tools.r8.dex.code;

import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.internal.AbstractC3650zs;
import com.android.tools.r8.internal.C0756Kt;
import com.android.tools.r8.internal.SJ;
import com.android.tools.r8.internal.WS;
import com.sun.tools.javac.jvm.ByteCodes;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes.dex */
public class J3 extends I0 {
    public J3(int i, B1 b1) {
        super(i, b1);
    }

    @Override // com.android.tools.r8.dex.code.I0, com.android.tools.r8.dex.code.A1
    public final /* bridge */ /* synthetic */ void a(C0421y c0421y, AbstractC3650zs abstractC3650zs, com.android.tools.r8.dex.M m, A5 a5, SJ sj) {
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String m() {
        return "ShrLong2Addr";
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final int p() {
        return ByteCodes.wide;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String u() {
        return "shr-long/2addr";
    }

    public J3(int i, int i2) {
        super(i, i2);
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final void a(C0756Kt c0756Kt) {
        WS ws = WS.f;
        byte b = this.f;
        c0756Kt.o(ws, b, b, this.g);
    }
}
