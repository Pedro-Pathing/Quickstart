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
public class Y extends G0 implements InterfaceC2592oc0 {
    public Y(int i, B1 b1) {
        super(i, b1);
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final boolean A() {
        return true;
    }

    @Override // com.android.tools.r8.dex.code.G0, com.android.tools.r8.dex.code.A1
    public final /* bridge */ /* synthetic */ void a(C0421y c0421y, AbstractC3650zs abstractC3650zs, com.android.tools.r8.dex.M m, A5 a5, SJ sj) {
    }

    @Override // com.android.tools.r8.dex.code.G0, com.android.tools.r8.dex.code.A1
    public final String b(C2108ja0 c2108ja0) {
        byte b = this.f;
        return b("v" + ((int) b) + ", " + Zf0.a(a(), 1) + " (" + a() + ")");
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final Y f() {
        return this;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String m() {
        return "Const4";
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final int p() {
        return 18;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String u() {
        return "const/4";
    }

    public Y(int i, int i2) {
        super(i, i2);
    }

    @Override // com.android.tools.r8.internal.InterfaceC2592oc0
    public int a() {
        return this.g;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String a(C2108ja0 c2108ja0) {
        byte b = this.f;
        return a("v" + ((int) b) + ", " + Zf0.a(a(), 2) + "  # " + a());
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final void a(C0756Kt c0756Kt) {
        int iA = a();
        c0756Kt.a(iA == 0 ? AbstractC3250vj0.p() : AbstractC3250vj0.o(), this.f, iA);
    }
}
