package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.D2;
import com.android.tools.r8.graph.S4;
import com.android.tools.r8.internal.AbstractC1308bC;
import com.android.tools.r8.internal.AbstractC3644zm0;
import com.android.tools.r8.internal.C0874Ot;
import com.android.tools.r8.internal.C1631ec;
import com.android.tools.r8.internal.C2052iw;
import com.android.tools.r8.internal.C2867rd;
import com.android.tools.r8.internal.H5;
import java.util.Map;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class K implements Y {
    public static final /* synthetic */ boolean c = true;
    public final C0421y a;
    public final Map b;

    public K(C0421y c0421y, Map map) {
        this.a = c0421y;
        this.b = map;
    }

    @Override // com.android.tools.r8.ir.optimize.Y
    public final boolean a() {
        return true;
    }

    @Override // com.android.tools.r8.ir.optimize.Y
    public final C0421y b() {
        return this.a;
    }

    @Override // com.android.tools.r8.ir.optimize.Y
    public final boolean a(C0874Ot c0874Ot, S4.c cVar, A5 a5, AbstractC3644zm0 abstractC3644zm0) {
        return true;
    }

    @Override // com.android.tools.r8.ir.optimize.Y
    public final A5 a(A5 a5, AbstractC1308bC abstractC1308bC) {
        T t = (T) this.b.get(abstractC1308bC);
        if (t != null) {
            return t.a;
        }
        return com.android.tools.r8.graph.H0.a(abstractC1308bC.g(this.a, a5));
    }

    @Override // com.android.tools.r8.ir.optimize.Y
    public final S a(C0874Ot c0874Ot, AbstractC1308bC abstractC1308bC, S4.c cVar, A5 a5, A5 a52, C1631ec c1631ec, C2052iw c2052iw, AbstractC3644zm0 abstractC3644zm0) {
        T t = (T) this.b.get(abstractC1308bC);
        if (t == null) {
            return null;
        }
        P p = new P();
        p.b = abstractC1308bC;
        p.e = t.a;
        if (a(this.a, p, abstractC1308bC, a5, a52)) {
            return p.a();
        }
        return null;
    }

    @Override // com.android.tools.r8.ir.optimize.Y
    public final boolean a(Q q, AbstractC3644zm0 abstractC3644zm0) {
        return true;
    }

    @Override // com.android.tools.r8.ir.optimize.Y
    public final boolean a(Q q, C0874Ot c0874Ot, C0874Ot c0874Ot2, AbstractC1308bC abstractC1308bC, H5 h5, AbstractC3644zm0 abstractC3644zm0) {
        return false;
    }

    @Override // com.android.tools.r8.ir.optimize.Y
    public final void a(C0874Ot c0874Ot) {
    }

    @Override // com.android.tools.r8.ir.optimize.Y
    public final C2867rd a(AbstractC1308bC abstractC1308bC, C2867rd c2867rd) {
        boolean z = c;
        if (!z && !abstractC1308bC.X1()) {
            throw new AssertionError();
        }
        T t = (T) this.b.get(abstractC1308bC.d0());
        if (!z && t == null) {
            throw new AssertionError();
        }
        D2 d2 = t.b;
        return d2 != null ? d2.getType().b(this.a).b() : c2867rd;
    }
}
