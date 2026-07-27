package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.internal.LN;
import java.util.Objects;

/* JADX INFO: renamed from: com.android.tools.r8.ir.optimize.h0, reason: case insensitive filesystem */
/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class C3676h0 extends AbstractC3674g0 {
    public final int c;

    public C3676h0(C3161ul0 c3161ul0, int i, LN ln) {
        super(c3161ul0, ln);
        this.c = i;
    }

    @Override // com.android.tools.r8.ir.optimize.AbstractC3674g0
    public final boolean a(int i) {
        return this.c == i;
    }

    public final boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null || C3676h0.class != obj.getClass()) {
            return false;
        }
        C3676h0 c3676h0 = (C3676h0) obj;
        return this.c == c3676h0.c && this.a == c3676h0.a && this.b == c3676h0.b;
    }

    public final int hashCode() {
        return Objects.hash(this.a, Integer.valueOf(this.c), this.b);
    }
}
