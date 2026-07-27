package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.internal.LN;
import java.util.Objects;

/* JADX INFO: renamed from: com.android.tools.r8.ir.optimize.i0, reason: case insensitive filesystem */
/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class C3678i0 extends AbstractC3674g0 {
    public final C3161ul0 c;

    public C3678i0(LN ln, C3161ul0 c3161ul0, C3161ul0 c3161ul02) {
        super(c3161ul0, ln);
        this.c = c3161ul02;
    }

    @Override // com.android.tools.r8.ir.optimize.AbstractC3674g0
    public final boolean a(int i) {
        return true;
    }

    public final boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null || C3678i0.class != obj.getClass()) {
            return false;
        }
        C3678i0 c3678i0 = (C3678i0) obj;
        return this.c == c3678i0.c && this.a == c3678i0.a && this.b == c3678i0.b;
    }

    public final int hashCode() {
        return Objects.hash(this.a, this.c, this.b);
    }
}
