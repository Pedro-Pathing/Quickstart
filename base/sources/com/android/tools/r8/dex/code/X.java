package com.android.tools.r8.dex.code;

import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.internal.AbstractC3250vj0;
import com.android.tools.r8.internal.AbstractC3650zs;
import com.android.tools.r8.internal.C0756Kt;
import com.android.tools.r8.internal.C2108ja0;
import com.android.tools.r8.internal.InterfaceC2592oc0;
import com.android.tools.r8.internal.SJ;
import com.android.tools.r8.internal.Zf0;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes.dex */
public class X extends M0 implements InterfaceC2592oc0 {
    public X(int i, B1 b1) {
        super(i, b1);
    }

    @Override // com.android.tools.r8.dex.code.M0, com.android.tools.r8.dex.code.A1
    public final /* bridge */ /* synthetic */ void a(C0421y c0421y, AbstractC3650zs abstractC3650zs, com.android.tools.r8.dex.M m, A5 a5, SJ sj) {
    }

    @Override // com.android.tools.r8.dex.code.M0, com.android.tools.r8.dex.code.A1
    public final String b(C2108ja0 c2108ja0) {
        short s = this.f;
        return b("v" + ((int) s) + ", " + Zf0.a((int) this.g, 4) + " (" + ((int) this.g) + ")");
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String m() {
        return "Const16";
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final int p() {
        return 19;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String u() {
        return "const/16";
    }

    public X(int i, int i2) {
        super(i, i2);
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final void a(C0756Kt c0756Kt) {
        short s = this.g;
        c0756Kt.a(s == 0 ? AbstractC3250vj0.p() : AbstractC3250vj0.o(), this.f, s);
    }

    @Override // com.android.tools.r8.internal.InterfaceC2592oc0
    public final int a() {
        return this.g;
    }
}
