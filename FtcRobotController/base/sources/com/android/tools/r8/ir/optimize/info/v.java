package com.android.tools.r8.ir.optimize.info;

import com.android.tools.r8.graph.C0257g1;
import com.android.tools.r8.internal.AbstractC0564Em;
import com.android.tools.r8.internal.B1;
import com.android.tools.r8.internal.C2758qS;
import com.android.tools.r8.internal.C3050tc0;
import com.android.tools.r8.internal.Dk0;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class v extends AbstractC3683e {
    public static final /* synthetic */ boolean e = true;
    public int b;
    public B1 a = Dk0.a;
    public int c = 0;
    public AbstractC0564Em d = AbstractC0564Em.m();

    public final v a(B1 b1, C0257g1 c0257g1) {
        if (!e) {
            b1.getClass();
            if ((b1 instanceof C3050tc0) && !c0257g1.getType().U0()) {
                throw new AssertionError();
            }
        }
        return a(b1);
    }

    @Override // com.android.tools.r8.ir.optimize.info.g
    public final g b() {
        return this;
    }

    @Override // com.android.tools.r8.ir.optimize.info.g
    public final v c() {
        return this;
    }

    @Override // com.android.tools.r8.ir.optimize.info.AbstractC3683e
    public final boolean e() {
        return (this.b & 1) != 0;
    }

    @Override // com.android.tools.r8.ir.optimize.info.AbstractC3683e
    public final B1 f() {
        return this.a;
    }

    @Override // com.android.tools.r8.ir.optimize.info.AbstractC3683e
    public final AbstractC0564Em g() {
        return this.d;
    }

    @Override // com.android.tools.r8.ir.optimize.info.AbstractC3683e
    public final int h() {
        return this.c;
    }

    @Override // com.android.tools.r8.ir.optimize.info.AbstractC3683e
    public final boolean i() {
        return (this.b & 2) != 0;
    }

    @Override // com.android.tools.r8.ir.optimize.info.AbstractC3683e
    public final boolean j() {
        return (this.b & 4) != 0;
    }

    public final v a(B1 b1) {
        if (!e && !this.a.isUnknown() && !b1.F()) {
            B1 b12 = this.a;
            b12.getClass();
            if (!(b12 instanceof C2758qS) || !this.a.m().a.K()) {
                throw new AssertionError();
            }
        }
        this.a = b1;
        return this;
    }
}
