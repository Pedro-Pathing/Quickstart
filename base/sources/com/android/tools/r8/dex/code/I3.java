package com.android.tools.r8.dex.code;

import com.android.tools.r8.internal.AbstractC2585oY;
import com.android.tools.r8.internal.C0756Kt;
import com.android.tools.r8.internal.Eb0;
import com.android.tools.r8.internal.Jl0;
import com.android.tools.r8.internal.WS;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes.dex */
public class I3 extends O0 {
    public I3(int i, B1 b1) {
        super(i, b1);
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final void a(C0756Kt c0756Kt) {
        WS ws = WS.e;
        short s = this.f;
        short s2 = this.g;
        byte b = this.h;
        boolean z = C0756Kt.D;
        if (z) {
            c0756Kt.getClass();
        } else {
            c0756Kt.getClass();
            if (!C0756Kt.b(ws)) {
                throw new AssertionError();
            }
        }
        Eb0 eb0 = new Eb0(ws, c0756Kt.a(s, 1, AbstractC2585oY.a(ws)), c0756Kt.b(s2, Jl0.a(ws)), c0756Kt.a(b));
        if (!z && eb0.i()) {
            throw new AssertionError();
        }
        c0756Kt.a(c0756Kt.u.g(), eb0);
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String m() {
        return "ShrIntLit8";
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final int p() {
        return 225;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final String u() {
        return "shr-int/lit8";
    }

    public I3(int i, int i2, int i3) {
        super(i, i2, i3);
    }
}
