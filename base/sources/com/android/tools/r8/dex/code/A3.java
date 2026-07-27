package com.android.tools.r8.dex.code;

import com.android.tools.r8.graph.C0309l1;
import com.android.tools.r8.graph.C0365q5;
import com.android.tools.r8.graph.Y5;
import com.android.tools.r8.internal.C0756Kt;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes.dex */
public class A3 extends AbstractC0162y3 implements InterfaceC0052d {
    public A3(int i, B1 b1, C0365q5 c0365q5) {
        super(i, b1, c0365q5.a());
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final void a(Y5 y5) {
        y5.a(this);
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final boolean h() {
        return true;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String m() {
        return "SgetWide";
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final int p() {
        return 97;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String u() {
        return "sget-wide";
    }

    public A3(int i, C0309l1 c0309l1) {
        super(i, c0309l1);
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final void a(C0756Kt c0756Kt) {
        c0756Kt.a(this.f, super.getField());
    }
}
