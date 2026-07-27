package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.AbstractC0410w3;
import com.android.tools.r8.graph.C0309l1;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.I2;
import com.android.tools.r8.internal.AbstractC0975So;
import com.android.tools.r8.shaking.C3877i;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class X {
    public static final /* synthetic */ boolean b = true;
    public final C0421y a;

    public X(C0421y c0421y) {
        this.a = c0421y;
    }

    public final O a(C0309l1 c0309l1, A5 a5) {
        AbstractC0410w3.a<?> aVarL = ((C3877i) this.a.g()).c(c0309l1).l();
        return aVarL == null ? O.c : a(aVarL.b, a5, com.android.tools.r8.graph.F0.a(aVarL.c, aVarL.d));
    }

    public final O a(com.android.tools.r8.graph.E0 e0, A5 a5, com.android.tools.r8.graph.G0 g0) {
        if (AbstractC0975So.a(e0, a5, this.a) && AbstractC0975So.a(g0, a5, this.a)) {
            I2 i2S = g0.s();
            if (!b && e0 == null) {
                throw new AssertionError();
            }
            return O.a(O.a(a5, e0.getType(), e0.getAccessFlags(), this.a), O.a(a5, i2S, g0.getAccessFlags(), this.a), (C0421y<?>) this.a);
        }
        return O.c;
    }
}
