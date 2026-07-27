package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.C0309l1;
import com.android.tools.r8.internal.C3161ul0;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class l0 {
    public static final /* synthetic */ boolean c = true;
    public final C0309l1 a;
    public final C3161ul0 b;

    public l0(C0309l1 c0309l1, C3161ul0 c3161ul0) {
        if (!c && c3161ul0 != c3161ul0.i()) {
            throw new AssertionError();
        }
        this.a = c0309l1;
        this.b = c3161ul0;
    }

    public final boolean equals(Object obj) {
        if (!(obj instanceof l0)) {
            return false;
        }
        l0 l0Var = (l0) obj;
        return l0Var.b == this.b && l0Var.a == this.a;
    }

    public final int hashCode() {
        return (this.a.hashCode() * 7) + this.b.b;
    }
}
