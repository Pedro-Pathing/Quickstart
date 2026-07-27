package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.internal.DS$$ExternalSyntheticLambda0;
import com.android.tools.r8.internal.LN;

/* JADX INFO: renamed from: com.android.tools.r8.ir.optimize.g0, reason: case insensitive filesystem */
/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public abstract class AbstractC3674g0 {
    public final C3161ul0 a;
    public final LN b;

    public AbstractC3674g0(C3161ul0 c3161ul0, LN ln) {
        this.a = c3161ul0;
        this.b = ln;
    }

    public static AbstractC3674g0 a(LN ln, C3161ul0 c3161ul0, C3161ul0 c3161ul02) {
        return c3161ul02.c(new DS$$ExternalSyntheticLambda0()) ? new C3676h0(c3161ul0, c3161ul02.m().H().N2(), ln) : new C3678i0(ln, c3161ul0, c3161ul02);
    }

    public abstract boolean a(int i);
}
