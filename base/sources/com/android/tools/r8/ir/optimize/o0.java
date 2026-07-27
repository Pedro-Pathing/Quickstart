package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.B1;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.I2;
import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.AbstractC3250vj0;
import com.android.tools.r8.internal.Bc0;
import com.android.tools.r8.internal.C0874Ot;
import com.android.tools.r8.internal.C2681pc0;
import com.android.tools.r8.internal.C2947sS;
import com.android.tools.r8.internal.C3050tc0;
import com.android.tools.r8.internal.C3143uc0;
import com.android.tools.r8.internal.K5;
import com.android.tools.r8.internal.R3;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class o0 implements m0 {
    public static final /* synthetic */ boolean c = true;
    public final Bc0 a;
    public final /* synthetic */ p0 b;

    public o0(p0 p0Var, Bc0 bc0) {
        this.b = p0Var;
        if (!c) {
            q0 q0Var = p0Var.i;
            int i = q0.e;
            if (!bc0.b(q0Var.a.V(), p0Var.a)) {
                throw new AssertionError();
            }
        }
        this.a = bc0;
    }

    @Override // com.android.tools.r8.ir.optimize.m0
    public final void a(K5 k5, AbstractC1076Vw abstractC1076Vw) {
        Bc0 bc0 = this.a;
        q0 q0Var = this.b.i;
        int i = q0.e;
        C0421y c0421yU = q0Var.a.U();
        C0874Ot c0874Ot = this.b.b;
        bc0.getClass();
        AbstractC1076Vw[] abstractC1076VwArrA = bc0.a(c0421yU, c0874Ot.i(), c0874Ot, abstractC1076Vw);
        if (!c && abstractC1076VwArrA.length != 1) {
            throw new AssertionError();
        }
        boolean z = R3.a;
        k5.a(abstractC1076VwArrA[0], this.b.d);
        this.b.h = true;
    }

    @Override // com.android.tools.r8.ir.optimize.m0
    public final AbstractC3250vj0 a(C0421y c0421y, AbstractC3250vj0 abstractC3250vj0) {
        B1 b1A = c0421y.a();
        if (!this.a.O()) {
            Bc0 bc0 = this.a;
            bc0.getClass();
            if (!(bc0 instanceof C2681pc0)) {
                if (this.a.K()) {
                    return AbstractC3250vj0.a(this.a.r().b.getType(), C2947sS.h(), (C0421y<?>) c0421y);
                }
                boolean z = c;
                if (!z) {
                    Bc0 bc02 = this.a;
                    bc02.getClass();
                    if (!(bc02 instanceof C3143uc0)) {
                        throw new AssertionError();
                    }
                }
                if (abstractC3250vj0.I()) {
                    if (!z) {
                        Bc0 bc03 = this.a;
                        bc03.getClass();
                        if (!(bc03 instanceof C3050tc0)) {
                            throw new AssertionError();
                        }
                    }
                    return AbstractC3250vj0.m();
                }
                if (z || abstractC3250vj0.H()) {
                    return abstractC3250vj0;
                }
                throw new AssertionError();
            }
        }
        I2 i2 = b1A.Y1;
        q0 q0Var = this.b.i;
        int i = q0.e;
        C0421y c0421y2 = q0Var.a;
        C2947sS c2947sSB = C2947sS.b();
        i2.getClass();
        return AbstractC3250vj0.a(i2, c2947sSB, (C0421y<?>) c0421y2);
    }
}
