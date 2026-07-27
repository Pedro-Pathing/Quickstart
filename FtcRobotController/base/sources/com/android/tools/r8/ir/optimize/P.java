package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.D2;
import com.android.tools.r8.internal.AbstractC1308bC;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class P {
    public D2 a;
    public AbstractC1308bC b;
    public U c;
    public boolean d;
    public A5 e;

    public final void a(U u) {
        this.c = u;
    }

    public final Q a() {
        Q q = new Q(this.e, this.b, this.c);
        D2 d2 = this.a;
        if (d2 != null) {
            q.e = d2;
        }
        if (this.d) {
            q.d = true;
        }
        return q;
    }
}
