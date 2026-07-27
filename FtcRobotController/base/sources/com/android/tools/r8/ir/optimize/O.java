package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.AbstractC0247g;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.I2;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public class O {
    public final N a;
    public final I2 b;
    public static final /* synthetic */ boolean e = true;
    public static final O c = new O(N.c);
    public static final O d = new O(N.h);

    public O(N n) {
        if (!e && n != N.c && n != N.h) {
            throw new AssertionError();
        }
        this.a = n;
        this.b = null;
    }

    public static O a(A5 a5, I2 i2, AbstractC0247g abstractC0247g, C0421y c0421y) {
        if (abstractC0247g.m()) {
            return d;
        }
        if (abstractC0247g.i()) {
            return a5.a().t1() ? AbstractC3665d0.a(a5.s(), i2, c0421y) ? new O(N.e, i2) : c : i2 == a5.s() ? new O(N.d, i2) : c;
        }
        if (abstractC0247g.l()) {
            return i2.D0().equals(a5.s().D0()) ? new O(N.f, i2) : c0421y.a(a5.s(), i2).d() ? new O(N.g, i2) : c;
        }
        return i2.D0().equals(a5.s().D0()) ? new O(N.f, i2) : c;
    }

    public final boolean equals(Object obj) {
        if (!(obj instanceof O)) {
            return false;
        }
        O o = (O) obj;
        return this.a.ordinal() == o.a.ordinal() && this.b == o.b;
    }

    public final int hashCode() {
        if (this.b == null) {
            return this.a.ordinal();
        }
        return this.b.f.hashCode() * this.a.ordinal();
    }

    public O(N n, I2 i2) {
        boolean z = e;
        if (!z && n == N.c) {
            throw new AssertionError();
        }
        if (!z && n == N.h) {
            throw new AssertionError();
        }
        if (!z && i2 == null) {
            throw new AssertionError();
        }
        this.a = n;
        this.b = i2;
    }

    public static O a(C0421y c0421y, I2 i2, A5 a5) {
        if (i2.I0()) {
            return a(c0421y, i2.a(1, c0421y.a()), a5);
        }
        if (i2.T0()) {
            return d;
        }
        com.android.tools.r8.graph.E0 e0D = c0421y.d(i2);
        if (e0D == null) {
            return c;
        }
        return a(a5, i2, e0D.f, c0421y);
    }

    /* JADX WARN: Removed duplicated region for block: B:156:0x01e8 A[RETURN] */
    /* JADX WARN: Removed duplicated region for block: B:157:0x01e9  */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public static com.android.tools.r8.ir.optimize.O a(com.android.tools.r8.ir.optimize.O r5, com.android.tools.r8.ir.optimize.O r6, com.android.tools.r8.graph.C0421y<?> r7) {
        /*
            Method dump skipped, instruction units count: 691
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.android.tools.r8.ir.optimize.O.a(com.android.tools.r8.ir.optimize.O, com.android.tools.r8.ir.optimize.O, com.android.tools.r8.graph.y):com.android.tools.r8.ir.optimize.O");
    }
}
