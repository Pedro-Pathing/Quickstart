package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.C3126uP;
import com.android.tools.r8.internal.C3161ul0;
import java.util.HashSet;
import java.util.function.Predicate;

/* JADX INFO: renamed from: com.android.tools.r8.ir.optimize.a0, reason: case insensitive filesystem */
/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class C3659a0 {
    public final HashSet a = new HashSet();
    public final com.android.tools.r8.ir.regalloc.f b;

    public C3659a0(com.android.tools.r8.ir.regalloc.f fVar) {
        this.b = fVar;
    }

    public final boolean a(AbstractC1076Vw abstractC1076Vw) {
        if (abstractC1076Vw.i2()) {
            C3126uP c3126uPN0 = abstractC1076Vw.n0();
            int iB = this.b.b(c3126uPN0.L2(), c3126uPN0.e);
            int iA = this.b.a(c3126uPN0.K2(), c3126uPN0.e);
            if (iB == iA) {
                return true;
            }
            for (C3126uP c3126uP : this.a) {
                int iB2 = this.b.b(c3126uP.L2(), c3126uP.e);
                int iA2 = this.b.a(c3126uP.K2(), c3126uP.e);
                if (iB2 == iB && iA2 == iA) {
                    return true;
                }
                if (iA2 == iB && iB2 == iA) {
                    if (!c3126uPN0.I2().b()) {
                        return true;
                    }
                    if (iB != iA + 1 && iB + 1 != iA) {
                        return true;
                    }
                }
            }
        }
        if (abstractC1076Vw.d() != null && abstractC1076Vw.d().T()) {
            final C3161ul0 c3161ul0D = abstractC1076Vw.d();
            final int iA3 = this.b.a(c3161ul0D, abstractC1076Vw.e);
            this.a.removeIf(new Predicate() { // from class: com.android.tools.r8.ir.optimize.a0$$ExternalSyntheticLambda0
                @Override // java.util.function.Predicate
                public final boolean test(Object obj) {
                    return this.f$0.a(c3161ul0D, iA3, (C3126uP) obj);
                }
            });
        }
        if (!abstractC1076Vw.i2()) {
            return false;
        }
        this.a.add(abstractC1076Vw.n0());
        return false;
    }

    public final boolean a(C3161ul0 c3161ul0, int i, C3126uP c3126uP) {
        int iB = this.b.b(c3126uP.L2(), c3126uP.e);
        int iA = this.b.a(c3126uP.K2(), c3126uP.e);
        for (int i2 = 0; i2 < c3161ul0.o.O(); i2++) {
            for (int i3 = 0; i3 < c3126uP.d().o.O(); i3++) {
                int i4 = i + i2;
                if (i4 == iA + i3 || i4 == iB + i3) {
                    return true;
                }
            }
        }
        return false;
    }
}
